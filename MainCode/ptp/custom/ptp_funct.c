/*frame ptp buffer
1)package ID
2)No. emitter
3)operation mode emitter
4)packet number in the message
5)ptp_state
6)delay_ns
7)offset_s
8)offset_ns
9)observedDrift
*/

#define NUM_PTP_BUF 9

static int32_t ptp_buf[NUM_PTP_BUF];

//functions
static uint32_t read_PTP(int32_t** buf, const PtpClock *ptpClock) {
	ptp_buf[4] = ptpClock->portDS.portState;
	ptp_buf[5] = ptpClock->portDS.peerMeanPathDelay.nanoseconds;
  switch (ptpClock->portDS.delayMechanism)
	{
		case E2E:
        ptp_buf[5] = ptpClock->currentDS.meanPathDelay.nanoseconds);
        break;
    case P2P:
        ptp_buf[5] = ptpClock->portDS.peerMeanPathDelay.nanoseconds);
        break;
	}
	ptp_buf[6] = ptpClock->currentDS.offsetFromMaster.seconds;
	ptp_buf[7] = ptpClock->currentDS.offsetFromMaster.nanoseconds;
	ptp_buf[8] = ptpClock->observedDrift;
}
