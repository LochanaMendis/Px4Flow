#include <stdio.h>
#include <unistd.h>
#include <mraa/i2c.h>

#define I2C_ADDR 0x42
float focal_length=0.016;
float constant=0.0006; 

mraa_i2c_context i2c;
mraa_i2c_context i2c1;
int16_t g_distance;
int16_t flowx;
int16_t flowy;
int16_t qual;

int16_t distance;
int16_t flowx_integral;
int16_t flowy_integral;
uint32_t integration_timespan;
uint8_t quality;
float T_x_previous=0;
float T_y_previous=0;
float acc_vx;
float acc_vy;
float V_y;
float V_x;

int moving_avg_countx=0;
int moving_avg_county=0;
float moving_avgx=0;
float moving_avgy=0;
float max_velocity=0;

float array8x[8]={0};
float array8y[8]={0};

int posx = 0;
float newAvgx = 0;
float sumx = 0;
//int lenx = sizeof(array8x[8]) / sizeof(float);
int lenx =8;

int posy = 0;
float newAvgy = 0;
float sumy = 0;
//int leny = sizeof(array8y[8]) / sizeof(float);
int leny =8;

void I2Cframe(){
	uint8_t buf[22];
	i2c1=mraa_i2c_init(1);
	mraa_i2c_address(i2c1,I2C_ADDR);
	mraa_i2c_write_byte(i2c1,0x00);
	mraa_i2c_read(i2c1, buf, 22);
		uint8_t frame_msb= buf[1];
		uint8_t frame_lsb= buf[0];
		uint8_t flowx_msb= buf[3];
		uint8_t flowx_lsb= buf[2];
		uint8_t flowy_msb= buf[5];
		uint8_t flowy_lsb= buf[4];
		uint8_t velocx_msb= buf[7];
		uint8_t velocx_lsb= buf[6];
		uint8_t velocy_msb= buf[9];
		uint8_t velocy_lsb= buf[8];
		uint8_t qual_msb= buf[11];
		uint8_t qual_lsb= buf[10];
		uint8_t gyrox_msb= buf[13];
		uint8_t gyrox_lsb= buf[12];
		uint8_t gyroy_msb= buf[15];
		uint8_t gyroy_lsb= buf[14];
		uint8_t gyroz_msb= buf[17];
		uint8_t gyroz_lsb= buf[16];
		uint8_t gyro_range= buf[18];
		uint8_t sonar_time= buf[19];
		uint8_t dist_msb= buf[21];
		uint8_t dist_lsb= buf[20];
		//uint8_t data = mraa_i2c_read_byte(i2c);
		//printf("Register= %d \n", data);
		//printf("msb %d \n lsb %d \n ", msb,lsb);
		g_distance=((unsigned int) dist_msb << 8) | (unsigned int) (dist_lsb);
		flowx=((unsigned int) flowx_msb << 8) | (unsigned int) (flowx_lsb);
		uint16_t frame_count=((unsigned int) frame_msb << 8) | (unsigned int) (frame_lsb);
		flowy=((unsigned int) flowy_msb << 8) | (unsigned int) (flowy_lsb);
		qual=((unsigned int) qual_msb << 8) | (unsigned int) (qual_lsb);
		int16_t velocx=((unsigned int) velocx_msb << 8) | (unsigned int) (velocx_lsb);
		int16_t velocy=((unsigned int) velocy_msb << 8) | (unsigned int) (velocy_lsb);
		 int16_t gyrox=((unsigned int) gyrox_msb << 8) | (unsigned int) (gyrox_lsb);
		 int16_t gyroy=((unsigned int) gyroy_msb << 8) | (unsigned int) (gyroy_lsb);
		 int16_t gyroz=((unsigned int) gyroz_msb << 8) | (unsigned int) (gyroz_lsb);
		printf("frame Count: %d \n", frame_count);
		printf("distance: %i \n", g_distance);
		printf("pixel_flow_x_sum: %d \n", flowx);
		printf("pixel_flow_y_sum: %d \n", flowy);
		printf("velocity comp x: %d \n", velocx);
		printf("velocity comp y: %d \n", velocy);
		printf("sonar_timestamp: %d \n", sonar_time);
		printf("gyro_x_rate: %f \n", gyrox);
		printf("gyro_y_rate: %f \n", gyroy);
		printf("gyro_z_rate: %f \n", gyroz);
		printf("quality: %d \n", qual);
		printf("\n");
	mraa_i2c_stop(i2c1);

} 
void I2C_Integral_frame(){
	uint8_t buf[30];
	i2c=mraa_i2c_init(1);
	mraa_i2c_address(i2c,I2C_ADDR);
	mraa_i2c_write_byte(i2c,0x16);
	mraa_i2c_read(i2c, buf, 30);
		uint8_t frame_msb= buf[1];
		uint8_t frame_lsb= buf[0];
	
	//mraa_i2c_write_byte(i2c,0x18);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t flowx_msb= buf[3];
		uint8_t flowx_lsb= buf[2];
	//mraa_i2c_write_byte(i2c,0x1A);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t flowy_msb= buf[5];
		uint8_t flowy_lsb= buf[4];
	//mraa_i2c_write_byte(i2c,0x1C);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t gyrox_msb= buf[7];
		uint8_t gyrox_lsb= buf[6];
	//mraa_i2c_write_byte(i2c,0x1E);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t gyroy_msb= buf[9];
		uint8_t gyroy_lsb= buf[8];
	//mraa_i2c_write_byte(i2c,0x20);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t gyroz_msb= buf[11];
		uint8_t gyroz_lsb= buf[10];
	//mraa_i2c_write_byte(i2c,0x22);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t integration_time_byte0= buf[12];
		uint8_t integration_time_byte1= buf[13];
		uint8_t integration_time_byte2= buf[14];
		uint8_t integration_time_byte3= buf[15];
	//mraa_i2c_write_byte(i2c,0x26);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t sonar_time_byte0= buf[16];
		uint8_t sonar_time_byte1= buf[17];
		uint8_t sonar_time_byte2= buf[18];
		uint8_t sonar_time_byte3= buf[19];
	//mraa_i2c_write_byte(i2c,0x2A);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t dist_msb= buf[21];
		uint8_t dist_lsb= buf[20];
	//mraa_i2c_write_byte(i2c,0x2C);
	//mraa_i2c_read(i2c, buf, 25);
		uint8_t gyrotemp_msb= buf[23];
		uint8_t gyrotemp_lsb= buf[22];
	//mraa_i2c_write_byte(i2c,0x2E);
	//mraa_i2c_read(i2c, buf, 25);
		 quality= buf[24];
	
		//uint8_t data = mraa_i2c_read_byte(i2c);
		//printf("Register= %d \n", data);
		//printf("msb %d \n lsb %d \n ", msb,lsb);
		distance=((unsigned int) dist_msb << 8) | (unsigned int) (dist_lsb);
		flowx_integral=((unsigned int) flowx_msb << 8) | (unsigned int) (flowx_lsb);
		uint16_t frame_count_integral=((unsigned int) frame_msb << 8) | (unsigned int) (frame_lsb);
		flowy_integral=((unsigned int) flowy_msb << 8) | (unsigned int) (flowy_lsb);
		 int16_t gyrox_integral=((unsigned int) gyrox_msb << 8) | (unsigned int) (gyrox_lsb);
		 int16_t gyroy_integral=((unsigned int) gyroy_msb << 8) | (unsigned int) (gyroy_lsb);
		 int16_t gyroz_integral=((unsigned int) gyroz_msb << 8) | (unsigned int) (gyroz_lsb);
		 integration_timespan=((unsigned int) integration_time_byte3 << 24) | ((unsigned int) integration_time_byte2 << 16)| ((unsigned int) integration_time_byte1 << 8) | (unsigned int) (integration_time_byte0);
		uint32_t sonar_timestamp=((unsigned int) sonar_time_byte3 << 24) | ((unsigned int) sonar_time_byte2 << 16)| ((unsigned int) sonar_time_byte1 << 8) | (unsigned int) (sonar_time_byte0);
		 int16_t gyro_temperature=((unsigned int) gyrotemp_msb << 8) | (unsigned int) (gyrotemp_lsb);
		printf("frame_count_integral: %d \n", frame_count_integral);
		printf("distance: %d \n", distance);
		printf("flowx_integral: %d \n", flowx_integral);
		printf("flowy_integral: %d \n", flowy_integral);
		printf("integration_timespan: %i \n", integration_timespan);
		printf("sonar_timestamp: %d \n", sonar_timestamp);
		printf("gyro_x_rate: %f \n", gyrox_integral);
		printf("gyro_y_rate: %f \n", gyroy_integral);
		printf("gyro_z_rate: %f \n", gyroz_integral);
		printf("gyro_temperature: %d \n", gyro_temperature);
		printf("quality: %d \n",quality);

		printf("\n");
	mraa_i2c_stop(i2c);

} 

float movingAvg(float *velArray, float *VelSum, int pos, int len, float nextVel)
{
  //Subtract the oldest number from the prev sum, add the new number
  *VelSum = *VelSum - velArray[pos] + nextVel;
  //Assign the nextNum to the position in the array
  velArray[pos] = nextVel;
  //return the average
  return *VelSum / len;
}

void distance_calculation(){
	 
         //int count = sizeof(sample) / sizeof(int);
	
	if (quality >100 ){
		float avg_gdistance=((float)distance)/1000; // in meters
		printf("avg_distance: %f \n",avg_gdistance);
		
		max_velocity=(1.5*avg_gdistance/100000);
		V_x=((float)flowx_integral*avg_gdistance/10000)/((float)integration_timespan/1000000);
		V_y=((float)flowy_integral*avg_gdistance/10000)/((float)integration_timespan/1000000);
		printf("V_x: %f \n",V_x);
		printf("V_y: %f \n",V_y);
		if (V_x<max_velocity){
			moving_avg_countx=moving_avg_countx+1;
			printf("moving_avg_count x: %d\n", moving_avg_countx);

			//velocityx[i]=V_x;
			
			newAvgx = movingAvg(array8x, &sumx, posx, lenx, V_x);
    			//printf("The new average x is %f\n", newAvgx);
   			posx++;
    			if (posx >= lenx){
      				posx = 0;
    			}
			//i++;
			if (moving_avg_countx>=8){
				acc_vx=acc_vx+(newAvgx);
				//printf("distance moved in x: %f \n",acc_vx);
				//printf("distance moved in y: %f \n",acc_vy);
				//printf("\n");
				
			}else {
				acc_vx=acc_vx+(sumx/moving_avg_countx);
				
			}

		}
		if (V_y<max_velocity){
			moving_avg_county=moving_avg_county+1;
			printf("moving_avg_count y: %d\n", moving_avg_county);

			//velocityx[i]=V_x;
			
			newAvgy = movingAvg(array8y, &sumy, posy, leny, V_y);
    			//printf("The new average y is %f\n", newAvgy);
   			posy=posy+1;
    			if (posy >= leny){
      				posy = 0;
    			}
			//i++;
			if (moving_avg_county>=8){
				acc_vy=acc_vy+(newAvgy);
				//printf("distance moved in y: %f \n",acc_vy);
				//printf("distance moved in y: %f \n",acc_vy);

				//printf("\n");
				
			}else {
				acc_vy=acc_vy+(sumy/moving_avg_county);
				
			}


		}/*
		for (int i = 0; i < sizeof(array8y) / sizeof(float); ++i){
   			printf("%f", array8y[i]);
		}
		printf("\n");
		//printf("array8y: %f \n",array8y[posy]);
		printf("sumy: %f \n",sumy);
		printf("posy: %i \n",posy);
		printf("leny: %i \n",leny);
		
		for (int i = 0; i < sizeof(array8x) / sizeof(float); ++i){
   			printf("%f", array8x[i]);
		}
		printf("\n");
		//printf("array8x: %f \n",array8y[posx]);
		printf("sumx: %f \n",sumx);
		printf("posx: %i \n",posx);
		printf("lenx: %i \n",lenx);*/


				
		printf("distance moved in y: %f \n",acc_vx);
		printf("The new average y is %f\n", newAvgx);
		printf("distance moved in x: %f \n",acc_vy);
		printf("The new average x is %f\n", newAvgy);
		printf("\n");	
	}
	
	

} 



int main() {
	mraa_init();
	//i2c=mraa_i2c_init(1);
	//mraa_i2c_address(i2c,I2C_ADDR);

	while (1){
		//I2Cframe();
		I2C_Integral_frame();
		distance_calculation();
		sleep(1);
		}
	
	return 0;
}
