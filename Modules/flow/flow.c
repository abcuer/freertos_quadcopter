#include "flow.h"
#include "usart.h"

FLOW_Struct mini;
PIXEL_FLOW_Struct pixel_flow;

float pixel_cpi = 1.0f; 
uint8_t Flow_SSI, Flow_SSI_CNT, Flow_Err;


void Flow_Parse_Data(uint8_t data) //串口1解析光流模块数据
{
	static uint8_t RxBuffer[32];
	static uint8_t _data_cnt = 0;
	static uint8_t state = 0; 
	uint8_t sum = 0;
	static uint8_t fault_cnt;
	
	// 0xFE 0x04 DATA0 DATA1 DATA2 DATA3 DATA4 DATA5 SUM SQUAL 0xAA
	switch(state)
	{
		case 0:
			if(data==0xFE)  //包头
			{
				state=1;
				RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
			
		case 1:
			if(data==0x04)
			{
				state=2;
				RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
			
		case 2:
			RxBuffer[_data_cnt++]=data;
			if(_data_cnt==11)
			{
				state = 0;
				_data_cnt = 0;
				sum =  (RxBuffer[2] + RxBuffer[3] + RxBuffer[4] + RxBuffer[5]+ RxBuffer[6] + RxBuffer[7]);
				// 0xFE 0x04 DATA0 DATA1 DATA2 DATA3 DATA4 DATA5 SUM SQUAL 0xAA
				if((0xAA == data) && (sum == RxBuffer[8])) 
				{
					Flow_SSI_CNT++;//光流数据频率
					
					//读取原始数据
					mini.flow_x = -((int16_t)(*(RxBuffer+3)<<8)|*(RxBuffer+2));
					mini.flow_y = ((int16_t)(*(RxBuffer+5)<<8)|*(RxBuffer+4));
					//mini.flow_y = -((s16)(*(RxBuffer+5)<<8)|*(RxBuffer+4));
					
					// 注意光流与加速度坐标系不一致，但与姿态角坐标系一致。 
					// 定点悬停时，光流内环pid运算输出值会赋值给期望姿态角，实现无人机位置调整。
					// 光流原始坐标  x+      最终光流坐标   x-	
					//             /                      /
					//      y+ ___/___ y-          y+ ___/___ y-
					//           /|                     /|
					//          / |                    / |
					//        x-	|                   x+ |
					//            z-                     z-  
					
					
					
					mini.qual = *(RxBuffer+9);
					mini.flow_High = ((int16_t)(*(RxBuffer+7)<<8)|*(RxBuffer+6));  //光流传输过来的数据为毫米，int16_t
					//旧光流模块 高度单位厘米，波特率19200，
					//新光流模块，高度单位毫米，波特率115200
					mini.flow_High = mini.flow_High/10.0; // flow_High转为 float，单位：厘米
					
					//式中HIGH为实际高度，单位：米；转移到原始数据积分前
					//	 cpi = ((FlightData.High.bara_height*0.01f) / 11.914f) *2.54f;
					pixel_cpi = ((50*0.01f) / 11.914f) *2.54f; //这里用的50cm处的cpi
					
					//单帧位移 高度融合(pixel_cpi ) 还是 总位移 高度融合（cpi）选择一个就可以了
					//不用的那个保持 默认值 1.0f 即可
					
					
					static float high_tmp=0; //高度滤波，计算光流cpi用高度high_tmp
					high_tmp += (mini.flow_High - high_tmp)*0.1; //
					pixel_cpi = pixel_cpi * ( 1.0f + (high_tmp-50.0f)/50.0f*0.1 ); //换算到真实高度下的cpi
					
					//if(high_tmp<10) pixel_cpi=0; //高度<10cm输出0，屏蔽光流输出
					
					
					//pixel_flow.fix_High = pixel_cpi; //上位机调试用
					
					mini.flow_x_iOUT += mini.flow_x ;       //这个位移单位为像素，没有换算为cm  
					mini.flow_y_iOUT += mini.flow_y ;       //这个位移输出调试用，后面的积分位移会被复位（用户拨动摇杆时）
					
					mini.flow_x = mini.flow_x * pixel_cpi;		//像素转为cm
					mini.flow_y = mini.flow_y * pixel_cpi;		//像素转为cm
					
					
					//积分要在这里（串口函数回调函数）做，避免丢失光流数据。
					//但每次收到的数据都要提前做好角度补偿和高度融合（cpi转化），而不能等积分后再做。
					mini.flow_x_i += mini.flow_x ; //积分出位移,cm单位，要在高度融合后才积分
					mini.flow_y_i += mini.flow_y ; 
					

					
					//判断光流数据是否有效
					if(mini.qual<25)  //0x19
					{
							fault_cnt++;													
							if(fault_cnt>60)	//连续60次异常标定为无效
							{
								fault_cnt = 60;
								mini.ok = 0;
							}
					}
					else 
					{
							fault_cnt=0;
							mini.ok = 1;
					}
				}
			}
		break;
			
		default:
			state = 0;
			_data_cnt = 0;
		break;
		
	}
}


void Pixel_Flow_set_zero(void) 
{
    mini.flow_x_i = 0;
    mini.flow_y_i = 0;
    pixel_flow.fix_x_i = 0;
    pixel_flow.fix_y_i = 0;
    pixel_flow.ang_x = 0;
    pixel_flow.ang_y = 0;
    pixel_flow.loc_x = 0;
    pixel_flow.loc_y = 0;
}

/**
 * @brief 光流数据融合与修正主函数
 * @param dT 调用周期（秒），如10ms调用则传入0.01f
 */
// 光流数据融合修正 (建议 10ms 调用一次)
void Pixel_Flow_Fix(EulerAngle_Struct *euler_angle,float dT) 
{
    // 异常处理
    if(Flow_Err == 1 || !mini.ok)
    {		
        mini.flow_x_i = mini.flow_y_i = 0;
        pixel_flow.fix_x = pixel_flow.fix_y = 0;
        pixel_flow.err1_cnt++;
        return;
    }

    // 预计算除法
    float inv_dt = 1.0f / dT;

    // 1. 位移低通滤波
    pixel_flow.fix_x_i += (mini.flow_x_i - pixel_flow.fix_x_i) * 0.2f;
    pixel_flow.fix_y_i += (mini.flow_y_i - pixel_flow.fix_y_i) * 0.2f;

    // 2. 姿态补偿计算：减少 tan() 调用
    // 600.0f 为根据焦距等参数确定的补偿系数
    float comp_target_x = 600.0f * tan(-euler_angle->pitch * ANG_2_RAD);
    float comp_target_y = 600.0f * tan(-euler_angle->roll * ANG_2_RAD);

    pixel_flow.ang_x += (comp_target_x - pixel_flow.ang_x) * 0.2f;
    pixel_flow.ang_y += (comp_target_y - pixel_flow.ang_y) * 0.2f;

    // 3. 融合补偿（计算最终位移）
    pixel_flow.out_x_i = pixel_flow.fix_x_i - (pixel_flow.ang_x * pixel_cpi);
    pixel_flow.out_y_i = pixel_flow.fix_y_i - (pixel_flow.ang_y * pixel_cpi);

    // 4. 微分求速度
    pixel_flow.x = (pixel_flow.out_x_i - pixel_flow.out_x_i_o) * inv_dt;
    pixel_flow.y = (pixel_flow.out_y_i - pixel_flow.out_y_i_o) * inv_dt;
    
    // 更新历史值
    pixel_flow.out_x_i_o = pixel_flow.out_x_i;
    pixel_flow.out_y_i_o = pixel_flow.out_y_i;

    // 5. 速度滤波
    pixel_flow.fix_x += (pixel_flow.x - pixel_flow.fix_x) * 0.1f;
    pixel_flow.fix_y += (pixel_flow.y - pixel_flow.fix_y) * 0.1f;

    // 6. 输出结果赋值
    pixel_flow.loc_x = pixel_flow.out_x_i;  // X 轴当前位置（相对于起飞点）   厘米 (cm)
    pixel_flow.loc_y = pixel_flow.out_y_i;  // Y 轴当前位置（相对于起飞点）
    pixel_flow.loc_xs = pixel_flow.fix_x;   // X 轴当前速度（水平移动速度）
    pixel_flow.loc_ys = pixel_flow.fix_y;   // Y 轴当前速度（水平移动速度）

    // 7. 坐标溢出保护：当位移超过 10m 时移动坐标轴
    if(fabs(pixel_flow.loc_x) > 1000.0f || fabs(pixel_flow.loc_y) > 1000.0f) {
        Pixel_Flow_set_zero();
    }
}

