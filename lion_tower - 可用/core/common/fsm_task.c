#include "fsm_task.h"

work_state_e work_state;
work_state_e work_state_last;
extern uint32_t time_tick_1ms;

object_t object;

int32_t time_tick_jump=0;
u8 jump_flag=0;

void fsm_task_init(void)
{
    work_state_last = STOP_STATE;
	  work_state = STOP_STATE;
	  memset(&object, 0, sizeof(object_t));
}
void work_state_fsm()
{
    work_state_last = work_state;

		switch(rc_msg.sw2){
		case RC_SW_UP:
		  work_state = RC_NORMAL_STATE;
			break;
    case RC_SW_MID:
			if(object.flag){
				work_state = AUTO_AIM_STATE;
			}
			else{
				work_state = AUTO_NOAIM_STATE;
			}
			break;
		case RC_SW_DOWN:
			work_state = STOP_STATE;
			break;
		default:
		  	break;
    }
}
