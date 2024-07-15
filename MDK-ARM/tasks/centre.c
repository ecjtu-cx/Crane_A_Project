#include "stdio.h"
#include "centre.h"
#include "PID.h"
#include "time.h"

//���������������,����0��1Ϊ����,3Ϊ��̨
static MOTOR motor[3];

//�����ӽ��г�ʼ��
void cheel_init(void)
{
	const float init_pid[3] = {0, 0, 0};//�趨P��I��D��������
	int now = 0, setmax = 0;
	int max_out = 0, max_iout = 0;
	for(int i = 0; i < 2; i ++){
		//ǰ���޷�����
		limit(now, setmax);
		//PID��ʼ������
		PID_init(&motor[i].pid, init_pid, max_out, max_iout);
	}
}

//�����ӽ��п���,����set��Ԥ�����ٶ�,time��ǰ��ʱ��
void motor_speed_control(int set, int time)
{
	time = clock() + time;
	while(clock() == time){
		int p1 = PID_calc(&motor[0].pid, motor[0].speed_rpm, set);
		int p2 = PID_calc(&motor[1].pid, motor[1].speed_rpm, set);
		CAN_cmd_chassis(p1, p2, 0, 0);
	}
	CAN_cmd_chassis(0, 0, 0, 0);
	for(int i = 0; i < 2; i ++){
		PID_clear(&motor[0].pid);
	}
}
