/*
 * ADS1220_setup.c
 *
 *  Created on: May 27, 2025
 *      Author: Admin
 */

#include "ADS1220_setup.h"


extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi3_rx;

#define ADC_BUFFER_SIZE 3
#define MAX_SAMPLES 600
#define VREF 1.8f

//static uint8_t adc_dma_buffer[ADC_BUFFER_SIZE];
extern volatile enum measurement_mode current_mode;
extern volatile uint8_t aquisition, hold;
extern volatile uint32_t selected_index;


const static float gainV[4] = {67.44439697, 7.65388393, 1001.44805908, 991.82720947};
const static float offsetV[4] = {-0.00735788, -0.01025916, -11.22583008, -10.58398628};
const static float gainI[2] = {0.27235544, 172.04113770};
const static float offsetI[2] = {-0.01464176, -8.62597656};


static int32_t adc_values[2];

volatile float v_adc_avg_values = 0;
volatile float i_adc_avg_values = 0;
volatile float v_adc_result_avg_values = 0;
volatile float i_adc_result_avg_values = 0;

volatile float v_adc_rms_values = 0;
volatile float i_adc_rms_values = 0;
volatile float v_adc_result_rms_values = 0;
volatile float i_adc_result_rms_values = 0;

volatile uint16_t samples_count = 0;

volatile float voltage_value = 0;
volatile float current_value = 0;
static uint32_t v_mode, i_mode;
ADS1220_Handler_t Handler;

// Chip select functions
void CS_UP(void)
{
	HAL_GPIO_WritePin(ADS1220_NCS_GPIO_Port, ADS1220_NCS_Pin, GPIO_PIN_SET);
}

void CS_DOWN(void)
{
	HAL_GPIO_WritePin(ADS1220_NCS_GPIO_Port, ADS1220_NCS_Pin, GPIO_PIN_RESET);
}

// Dummy transmit (needed for DMA read)
void TRANSMIT(uint8_t data)
{
    HAL_SPI_Transmit(&hspi3, &data, 1, HAL_MAX_DELAY);
}

uint8_t RECEIVE(void)
{
    uint8_t r;
    HAL_SPI_Receive(&hspi3, &r, 1, HAL_MAX_DELAY);
    return r;
}

void DELAY(uint32_t us)
{
    // Use HAL_Delay for ms or custom microsecond delay
    timerDelay_us(us);
}

uint8_t TRANSMIT_RECEIVE(uint8_t data)
{
    uint8_t r;
    HAL_SPI_TransmitReceive(&hspi3, &data, &r, 1, HAL_MAX_DELAY);
    return r;
}

uint8_t DRDY_Read(void)
{
    return HAL_GPIO_ReadPin(ADS1220_NDRDY_GPIO_Port, ADS1220_NDRDY_Pin);

}

// Hàm này cho sử dụng Interrupt của NDRDY
void ADS1220_NDRDY_HAL_GPIO_EXTI_Callback(void)
{
	// Cái này khoai.
//	if(is_reading)
//	{
//		CS_DOWN();
//		timerDelay_us(10);
//		 HAL_SPI_Receive_DMA(&hspi3, adc_dma_buffer, ADC_BUFFER_SIZE);
//		 timerDelay_us(10);
//
//	}


}


// Hàm này cho sử DMA nhưng khoai :)))

void ADS1220_HAL_SPI_RxCpltCallback(void)
{

	// Cái này khoai :)))
	// để dọc được 2 giá trị từ 2 kênh thì phải gửi chuyển kênh :))
//	if(is_reading)
//		{
//	CS_UP();
//	timerDelay_us(10);
//	int32_t raw = ((int32_t)(adc_dma_buffer[0] << 16 | adc_dma_buffer[1] << 8 | adc_dma_buffer[2]) << 8) >> 8;
//
//	adc_values[1] = raw;
//	voltage_value = ADCValueToVoltage(raw, 1.8f, 1); // 1.8V external ref, gain = 1
//
//		}

}


void ADS1220_power_down_for_idle(void)
{
	ADS1220_PowerDown(&Handler);
}

void ADS1220_change_and_config_adc_channel_1_for_current_measurement(void)
{
	i_mode = selected_index - 4;

	ADS1220_Parameters_t current_params = {
	        .InputMuxConfig = P0N1,
	        .GainConfig = _1_,
	        .PGAdisable = false,
	        .DataRate = _600_SPS_,
	        .OperatingMode = NormalMode,
	        .ConversionMode = 1,
	        .TempeSensorMode = 0,
	        .BurnOutCurrentSrc = 0,
	        .VoltageRef = ExternalREF0,
	        .FIRFilter = No50or60Hz,
	        .LowSidePwr = 0,
	        .IDACcurrent = Off,
	        .IDAC1routing = Disabled,
	        .IDAC2routing = Disabled,
	        .DRDYMode = 0
	    };

	ADS1220_ChangeConfig(&Handler, &current_params);
	ADS1220_ActivateContinuousMode(&Handler);
	ADS1220_StartSync(&Handler);
	samples_count = 0;

}

void ADS1220_change_and_config_adc_channel_2_for_voltage_measurement(void)
{
	v_mode = selected_index;
	ADS1220_Parameters_t voltage_params = {
		        .InputMuxConfig = P2N3,
		        .GainConfig = _1_,
		        .PGAdisable = false,
		        .DataRate = _600_SPS_,
		        .OperatingMode = NormalMode,
		        .ConversionMode = 1,
		        .TempeSensorMode = 0,
		        .BurnOutCurrentSrc = 0,
		        .VoltageRef = ExternalREF0,
		        .FIRFilter = No50or60Hz,
		        .LowSidePwr = 0,
		        .IDACcurrent = Off,
		        .IDAC1routing = Disabled,
		        .IDAC2routing = Disabled,
		        .DRDYMode = 0
		    };

	ADS1220_ChangeConfig(&Handler, &voltage_params);
	ADS1220_ActivateContinuousMode(&Handler);
	ADS1220_StartSync(&Handler);
	samples_count = 0;


}

void ADS1220_read_current_from_adc_channel_1(void)
{

	if(!HAL_GPIO_ReadPin(ADS1220_NDRDY_GPIO_Port, ADS1220_NDRDY_Pin) )
	{
		samples_count++;
		ADS1220_ReadData(&Handler, &adc_values[0]);

		i_adc_avg_values += adc_values[0];
		i_adc_rms_values += adc_values[0] * adc_values[0];

		if(samples_count >= MAX_SAMPLES)
		{

			if(!aquisition)
			{
				i_adc_result_avg_values = (float) i_adc_avg_values / MAX_SAMPLES;
				current_value = (gainI[i_mode] * ADCValueToVoltage(i_adc_result_avg_values, VREF, 1)) + offsetI[i_mode];
			}
			else
			{
				i_adc_result_rms_values = sqrtf( (float) i_adc_rms_values / MAX_SAMPLES);
				current_value = (gainI[i_mode] * ADCValueToVoltage(i_adc_result_rms_values, VREF, 1)) + offsetI[i_mode];
			}


			i_adc_avg_values = 0;
			i_adc_rms_values = 0;
			samples_count = 0;

			push_event(EVENT_CURRENT_VALUE_READY);

		}



	}
}


void ADS1220_read_voltage_from_adc_channel_2(void)
{
	if(!HAL_GPIO_ReadPin(ADS1220_NDRDY_GPIO_Port, ADS1220_NDRDY_Pin) )
	{
		samples_count++;
		ADS1220_ReadData(&Handler, &adc_values[1]);

		v_adc_avg_values += adc_values[1];
		v_adc_rms_values += adc_values[1] * adc_values[1];

		if(samples_count >= MAX_SAMPLES)
		{
			if(!aquisition)
			{
				v_adc_result_avg_values = (float) v_adc_avg_values / MAX_SAMPLES;
				voltage_value = (gainV[v_mode] * ADCValueToVoltage(v_adc_result_avg_values, VREF, 1)) + offsetV[v_mode];
			}
			else
			{
				v_adc_result_rms_values = sqrtf( (float) v_adc_rms_values / MAX_SAMPLES);
				voltage_value = (gainV[v_mode] * ADCValueToVoltage(v_adc_result_rms_values, VREF, 1)) + offsetV[v_mode];

			}


			v_adc_avg_values = 0;
			v_adc_rms_values = 0;
			samples_count = 0;

			push_event(EVENT_VOLTAGE_VALUE_READY);

		}


		voltage_value = (gainV[v_mode] * ADCValueToVoltage(adc_values[1], VREF, 1)) + offsetV[v_mode];
	}


}



void ADS1220_config_init_start(void) {
    Handler.ADC_CS_HIGH = CS_UP;
    Handler.ADC_CS_LOW = CS_DOWN;
    Handler.ADC_Transmit = TRANSMIT;
    Handler.ADC_Receive = RECEIVE;
    Handler.ADC_TransmitReceive = TRANSMIT_RECEIVE;
    Handler.ADC_DRDY_Read = DRDY_Read;
    Handler.ADC_Delay_US = DELAY;

    ADS1220_Parameters_t params = {
        .InputMuxConfig = P0N1,
        .GainConfig = _1_,
        .PGAdisable = false,
        .DataRate = _600_SPS_,
        .OperatingMode = NormalMode,
        .ConversionMode = 1,
        .TempeSensorMode = 0,
        .BurnOutCurrentSrc = 0,
        .VoltageRef = ExternalREF0,
        .FIRFilter = No50or60Hz,
        .LowSidePwr = 0,
        .IDACcurrent = Off,
        .IDAC1routing = Disabled,
        .IDAC2routing = Disabled,
        .DRDYMode = 0
    };

    ADS1220_Init(&Handler, &params);
    ADS1220_ActivateContinuousMode(&Handler);
    ADS1220_StartSync(&Handler);
    samples_count = 0;

}


