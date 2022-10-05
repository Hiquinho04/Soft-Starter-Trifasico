//INCLUDES --
#include "stm32f4xx_hal.h"
#include "stdio.h"


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "times.h"
#include "math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int times_of_start(int time){

	//ARM frequency oscillation
	unsigned long int fosc;
	unsigned long int calculo;
	int arr;
	unsigned long int time_1;

	time_1 = (unsigned long int)(time * 1000);
	fosc = (84 * 1000); //84*10^3

	//PSC +1 =  10000
	//time esta na base de segundos
	//time_1 esta na base de milisegundos
	//arr+1 = 84*10^3 * time_1/psc+1

	//VALOR ARR PARA TEMPO FINAL
	calculo = (fosc * time_1)/10000;
	//VALOR ARR PARA QUE CADA SOMADO TODAS AS RESOLUÇÕES, DE O TEMPO FINAL
	arr = (int)((calculo-1)/75);

	return arr;

}
