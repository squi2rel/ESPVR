
/* DSI-TX1 Interrupt Status */
#define TC_MSK_DSITX1_INT 11
/*Audio Mute Interrupt Status
   0: Normal
   1: Audio change from Normal to Mute
Default = 0 (Value immediately becomes '1' after reset)  */
#define TC_MSK_AMUTE_INT 10
/* HDMI-RX Interrupt Status
Note: all HDMI interrupt flags defined in HDMI register space */
#define TC_MSK_HDMI_INT 9
/* DSI-TX0 Interrupt Status */
#define TC_MSK_DSITX0_INT 8
/* TC358870XBG System Interrupt Status
   0: Normal
   1: Video/Audio Overflow/Underflow/WakeUp occurs */
#define TC_MSK_SYS_INT 5
/* CEC Error Interrupt Status
   0: Normal
   1: CEC Errors occurs */
#define TC_MSK_CEC_EINT 4
/* CEC Transmit Interrupt Status
   0: Idle
   1: Transmit completed/done */
#define TC_MSK_CEC_TINT 3
/* CEC Receive Interrupt Status
   0: Idle
   1: Data Received */
#define TC_MSK_CEC_RINT 2
/* IR Error Interrupt Status
   0: No Error
   1: Error occurs (overflow error) */
#define TC_MSK_IR_EINT 1
/* IR Data Interrupt Status
   0: Idle
   1: Interrupt occurs (IR Data available) */
#define TC_MSK_IR_DINT 0