#ifndef __PID_H__
#define __PID_H__

// PID²ÎÊý
// PID �����ṹ��
#define Limit(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
typedef struct
{
    float kp;         // ��������  Ӱ����Ӧ�ٶȣ��������𵴣���С��Ӧ��
    float ki;         // ��������  ������̬���
    float kd;         // ΢������  ���ٳ�������������
    float desire;     // Ŀ��ֵ
    float measure;    // ����ֵ��ʵ�ʷ���ֵ��
    float integral;   // �������ۼ�
    float last_error; // ��һ�ε����
    float output;     // PID �������ֵ
} PID_Struct;

void PID_Reset(PID_Struct *pid);
void PID_Calculate(PID_Struct *pid, float dt);
void PID_Cascade(PID_Struct *outter, PID_Struct *inner, float dt);

#endif /* __PID_H__ */
