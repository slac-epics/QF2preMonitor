/*
 * QF2pre board monitors
 */
#include <string.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>
#include <asynOctet.h>
#include <asynInt32.h>
#include <drvAsynIPPort.h>

#define QF2PRE_MONITOR_UDP_PORT     50001

#define QF2PRE_MONITOR_TX_COUNT     (2*63)
#define QF2PRE_MONITOR_RX_CAPACITY  500

#define COMMAND_RETRY_LIMIT         4

#define ASYN_SUBADDRESS_STATISTICS  0x2000

/*
 * ASYN subaddress decoding
 * Short: Least significant 12 bits are index into received packet.
 *        Next three bits are number of valid bits in high byte, minus 1.
 *        Next bit is set if value is signed.
 * Boolean: Least significant 12 bits are index into received packet.
 *          Next three bits are bit number
 */
#define A_IS_BOOLEAN(a)  ((a) & 0x10000)
# define A_BITNUMBER(a)  (((a) >> 12) & 0x7)
#define A_IS_SIGNED(a)   ((a) & 0x8000)
# define A_HIGH_COUNT(a) ((((a) >> 12) & 0x7) + 1)
#define A_INDEX(a)       ((a) & 0xFFF)

/*
 * Link to lower port
 */
typedef struct portLink {
    char             *portName;
    char             *hostName;
    asynUser         *pasynUser;
    int               isCommunicating;
} portLink;

/*
 * Driver private storage
 */
typedef struct drvPvt {
    /*
     * Link to lower-level port
     */
    portLink        udpLink;

    /*
     * Asyn interfaces we provide
     */
    char           *portName;
    asynInterface   asynCommon;
    asynInterface   asynOctet;
    void           *asynOctetInterruptPvt;
    asynInterface   asynInt32;
    void           *asynInt32InterruptPvt;

    /*
     * I/O buffers
     */
    const char      txBuf[QF2PRE_MONITOR_TX_COUNT];
    char            rxBuf[QF2PRE_MONITOR_RX_CAPACITY];
    int             rxFullWarned;

    /*
     * Statistics
     */
    unsigned long   commandCount[COMMAND_RETRY_LIMIT+1];
    unsigned long   commandFailedCount;
} drvPvt;

static void
byteReverse(char *cp, size_t n)
{
    char *ep = cp + n - 1;

    while (cp < ep) {
        char t = *cp;
        *cp++ = *ep;
        *ep-- = t;
    }
}

static void
processSystemMonitorPacket(drvPvt *pdpvt, int nRead)
{
    ELLLIST *pclientList;
    interruptNode *pnode;
    epicsTimeStamp now;


    epicsTimeGetCurrent(&now);
    if (nRead >= 2) byteReverse(pdpvt->rxBuf, nRead);
    pasynManager->interruptStart(pdpvt->asynInt32InterruptPvt, &pclientList);
    pnode = (interruptNode *)ellFirst(pclientList);
    while (pnode) {
        asynInt32Interrupt *int32Interrupt = pnode->drvPvt;
        unsigned int a = int32Interrupt->addr;
        int aIndex = A_INDEX(a);
        int value;
        pnode = (interruptNode *)ellNext(&pnode->node);

        if (aIndex < nRead) {
            unsigned char lo = pdpvt->rxBuf[aIndex];
            if (A_IS_BOOLEAN(a)) {
                value = lo & (1 << A_BITNUMBER(a));
            }
            else {
                int highCount = A_HIGH_COUNT(a);
                unsigned char hi = pdpvt->rxBuf[aIndex+1];
                int highMask = ~(~0 << highCount);
                value = ((hi & highMask) << 8) | lo;
                if (A_IS_SIGNED(a) && (value & (1 << (highCount + 7)))) {
                    value -= (1 << (highCount + 8));
                }
            }
            int32Interrupt->pasynUser->auxStatus = asynSuccess;
        }
        else {
            value = 0;
            int32Interrupt->pasynUser->auxStatus = asynError;
        }
        int32Interrupt->pasynUser->timestamp = now;
        int32Interrupt->callback(int32Interrupt->userPvt,
                                                     int32Interrupt->pasynUser,
                                                     value);
    }
    pasynManager->interruptEnd(pdpvt->asynInt32InterruptPvt);
}

/*
 * asynCommon methods
 */
static void
report(void *pvt, FILE *fp, int details)
{
    drvPvt *pdpvt = (drvPvt *)pvt;
    int i;

    for (i = 0 ; i < COMMAND_RETRY_LIMIT ; i++) {
        if (pdpvt->commandCount[i]) {
            fprintf(fp, "%4d: %9lu\n", i, pdpvt->commandCount[i]);
        }
    }
    if (pdpvt->commandFailedCount) {
        fprintf(fp, "FAIL: %9lu\n", pdpvt->commandFailedCount);
    }
}

static asynStatus
connect(void *pvt, asynUser *pasynUser)
{
    pasynManager->exceptionConnect(pasynUser);
    return asynSuccess;
}

static asynStatus
disconnect(void *pvt, asynUser *pasynUser)
{
    pasynManager->exceptionDisconnect(pasynUser);
    return asynSuccess;
}
static asynCommon commonMethods = { report, connect, disconnect };

/*
 * asynOctet methods -- none -- all handled by interrup calllback
 */
static asynOctet octetMethods = { NULL, NULL };

/*
 * asynInt32 methods
 */
static asynStatus
int32Read(void *pvt, asynUser *pasynUser, epicsInt32 *value)
{
    drvPvt *pdpvt = (drvPvt *)pvt;
    asynStatus status;
    int address;
    size_t nWrite, nRead;
    int eomReason;
    int retry = 0;

    if ((status = pasynManager->getAddr(pasynUser, &address)) != asynSuccess)
        return status;
    if (address & ASYN_SUBADDRESS_STATISTICS) {
        int idx = address & ~ASYN_SUBADDRESS_STATISTICS;
       if (idx <= COMMAND_RETRY_LIMIT)
           *value = pdpvt->commandCount[idx];
       else if (idx == COMMAND_RETRY_LIMIT + 1)
           *value = pdpvt->commandFailedCount;
       else
           status = asynError;
        return status;
    }
    for (;;) {
        status = pasynOctetSyncIO->writeRead(pdpvt->udpLink.pasynUser,
                         pdpvt->txBuf, sizeof pdpvt->txBuf,
                         pdpvt->rxBuf, sizeof pdpvt->rxBuf,
                         1.0,
                         &nWrite, &nRead, &eomReason);
        if (status == asynSuccess) {
            if ((nRead == sizeof pdpvt->rxBuf) && !pdpvt->rxFullWarned) {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "***** WARNING -- Read from %s reached buffer capacity.  "
                    "This will likely mangle system monitor values!\n",
                                                      pdpvt->udpLink.portName);
                pdpvt->rxFullWarned++;
            }
            pdpvt->commandCount[retry]++;
            break;
        }
        if (++retry > COMMAND_RETRY_LIMIT) {
            pdpvt->commandFailedCount++;
            break;
        }
    }
    processSystemMonitorPacket(pdpvt, (status == asynSuccess) ? nRead : -1);
    return status;
}
static asynInt32 int32Methods = { NULL, int32Read };

static void
qf2preMonConfigure(const char *portName, const char *hostInfo, int priority)
{
    drvPvt *pdpvt;
    asynStatus status;
    int i;

    /*
     * Handle defaults
     */
    if (priority == 0) priority = epicsThreadPriorityMedium;
    if ((portName == NULL) || (*portName == '\0')
     || (hostInfo == NULL)  || (*hostInfo == '\0')) {
        printf("Required argument not present!\n");
        return;
    }

    /*
     * Sanity checks
     */
    if (strchr(hostInfo, ':') != NULL) {
        printf("Host info must not specify port.\n");
        return;
    }
    if (strchr(hostInfo, '*') != NULL) {
        printf("Host info must not specify broadcast.\n");
        return;
    }
    if (strchr(hostInfo, ' ') != NULL) {
        printf("Host info must not specify protocol.\n");
        return;
    }

    /*
     * Set up local storage
     */
    pdpvt = (drvPvt *)callocMustSucceed(1, sizeof(drvPvt), "qf2preMonConf");
    pdpvt->portName = epicsStrDup(portName);

    /*
     * Create the port that we'll use for I/O.
     * Configure it with our priority, autoconnect, no process EOS.
     */
    i = strlen(portName) + 5;
    pdpvt->udpLink.portName = callocMustSucceed(1, i, "qf2preMonConf");
    sprintf(pdpvt->udpLink.portName, "%s_UDP", portName);
    i = strlen(hostInfo) + 20;
    pdpvt->udpLink.hostName = callocMustSucceed(1, i, "qf2preMonConf");
    sprintf(pdpvt->udpLink.hostName, "%s:%d UDP", hostInfo,
                                                       QF2PRE_MONITOR_UDP_PORT);
    drvAsynIPPortConfigure(pdpvt->udpLink.portName, 
                           pdpvt->udpLink.hostName, priority, 0, 1);
    status = pasynOctetSyncIO->connect(pdpvt->udpLink.portName, -1,
                                               &pdpvt->udpLink.pasynUser, NULL);
    if (status != asynSuccess) {
        printf("Can't connect to \"%s\"\n", pdpvt->udpLink.portName);
        return;
    }

    /*
     * Create our port
     */
    status = pasynManager->registerPort(portName,
                                        ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                                        1,         /*  autoconnect */
                                        priority,  /* priority */
                                        0);        /* default stack size */
    if (status != asynSuccess) {
        printf("Can't register port %s", portName);
        return;
    }

    /*
     * Advertise our interfaces
     */
    pdpvt->asynCommon.interfaceType = asynCommonType;
    pdpvt->asynCommon.pinterface  = &commonMethods;
    pdpvt->asynCommon.drvPvt = pdpvt;
    status = pasynManager->registerInterface(portName, &pdpvt->asynCommon);
    if (status != asynSuccess) {
        printf("Can't register asynCommon support.\n");
        return;
    }
    pdpvt->asynOctet.interfaceType = asynOctetType;
    pdpvt->asynOctet.pinterface = &octetMethods;
    pdpvt->asynOctet.drvPvt = pdpvt;
    status = pasynOctetBase->initialize(portName, &pdpvt->asynOctet, 0, 0, 0);
    if (status != asynSuccess) {
        printf("Can't register asynOctet support.\n");
        return;
    }
    pasynManager->registerInterruptSource(portName, &pdpvt->asynOctet,
                                                &pdpvt->asynOctetInterruptPvt);
    pdpvt->asynInt32.interfaceType = asynInt32Type;
    pdpvt->asynInt32.pinterface = &int32Methods;
    pdpvt->asynInt32.drvPvt = pdpvt;
    status = pasynInt32Base->initialize(portName, &pdpvt->asynInt32);
    if (status != asynSuccess) {
        printf("Can't register asynInt32 support.\n");
        return;
    }
    pasynManager->registerInterruptSource(portName, &pdpvt->asynInt32,
                                                &pdpvt->asynInt32InterruptPvt);
}

/*
 * IOC shell command registration
 */
static const iocshArg monCnfgArg0 ={ "port",     iocshArgString};
static const iocshArg monCnfgArg1 ={ "address",  iocshArgString};
static const iocshArg monCnfgArg2 ={ "priority", iocshArgInt};
static const iocshArg *monCnfgArgs[] = {
                                     &monCnfgArg0, &monCnfgArg1, &monCnfgArg2 };
static const iocshFuncDef qf2preMonCnfgFuncDef =
                                  {"qf2preMonitorConfigure", 3, monCnfgArgs};
static void qf2preMonCnfgCallFunc(const iocshArgBuf *args)
{
    qf2preMonConfigure(args[0].sval, args[1].sval, args[2].ival);
}

static void
qf2preMonitorRegisterCommands(void)
{
    iocshRegister(&qf2preMonCnfgFuncDef, qf2preMonCnfgCallFunc);
}
epicsExportRegistrar(qf2preMonitorRegisterCommands);
