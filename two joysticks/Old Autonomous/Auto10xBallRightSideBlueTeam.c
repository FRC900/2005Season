//INSIDE relay.c
void Int_3_Handler(unsigned char RB4_State, unsigned char *hitBall)
{
	unsigned char Bryan; // temporary data buffer
	unsigned int Int_3_Period; // pulse-width period
	unsigned int Int_3_Up_Edge_Count_Low; // lower 16-bits of the rising-edge time snapshot
	unsigned int Int_3_Up_Edge_Count_High; // upper 16-bits of the rising-edge time snapshot
	static unsigned int Int_3_Down_Edge_Count_Low; // lower 16-bits of the falling-edge time snapshot
	static unsigned int Int_3_Down_Edge_Count_High; // upper 16-bits of the falling-edge time snapshot
	

	if(!*hitBall)
	{
		switch(RB4_State) // current state determines how the function behaves
		{
		case 0: // falling-edge detected (beginning of the pulse)
			Int_3_Down_Edge_Count_High = Clock; // get a snapshot of the time
			Bryan = TMR1L; // TMR1L must be read before TMR1H
			Int_3_Down_Edge_Count_Low = TMR1H;
			Int_3_Down_Edge_Count_Low <<= 8;
			Int_3_Down_Edge_Count_Low += Bryan;
			break; // now wait for the rising-edge interrupt to happen...
	
		case 1: // rising-edge detected (end of the pulse)
			Int_3_Up_Edge_Count_High = Clock; // get a snapshot of the time
			Bryan = TMR1L;
			Int_3_Up_Edge_Count_Low = TMR1H;
			Int_3_Up_Edge_Count_Low <<= 8;
			Int_3_Up_Edge_Count_Low += Bryan;
			
			// determine the pulse-width period by determining the time 
			// difference between the falling-edge and rising-edge interrupts
			if (Int_3_Up_Edge_Count_High == Int_3_Down_Edge_Count_High)
			{
				// this is quicker because the 16-bit system clock hasn't changed and therefore has no effect on the outcome
				Int_3_Period = Int_3_Up_Edge_Count_Low - Int_3_Down_Edge_Count_Low;
			}	
			else
			{
				// this works because the pulse-width will always be less than one clock tick (= 65536 timer ticks)
				Int_3_Period = 65536 - Int_3_Down_Edge_Count_Low + Int_3_Up_Edge_Count_Low;
			}
		
			// now catagorize the pulse type and update the associated statistics variable
			if ((Int_3_Period >= BEACON_0_LOWER_BOUND) && (Int_3_Period <= BEACON_0_UPPER_BOUND))
			{
				*hitBall=1;
			}
			break; // now wait for another falling-edge interrupt to happen...
		}
	}
	else
	{
		Disable_Receiver();
	}
}
//INSIDE user_routines_fast.c
void User_Autonomous_Code(void)
{
  unsigned char task=0;
  unsigned int old_clock[7];
  unsigned int clock=0;
  unsigned char old_pwmleft=127;
  unsigned char old_pwmright=127;

  for(task;task<7;task++)
  {
  	old_clock[task]=0;
  }
  task=0;

  while(autonomous_mode)
  {
    if(statusflag.NEW_SPI_DATA) //26.2ms loop
    {
        Getdata(&rxdata);   // bad things will happen if you move or delete this
		if(hitBall)
		{
			if(clock==0)							//See the Light
			{
				task=2;
				clock++;
			}
			else
			{
				if(clock<20)
				{
					task=3;							//Hit the Ball
					clock++;
				}
				else
				{
					if(clock<100)
					{
						task=4;						//Pull Arm Back
						clock++;
					}
					else
					{
						if(clock<330)
						{
							task=5;					//Go Forward Again
							clock++;
						}
						else
						{
							task=6;					//STOP
							clock++;
						}
					}
				}
			}
		}
		else
		{
			task=0;
		}
		
		if((clock&0xf)==0) printf("Task = %d\n",(int)task);
		if((clock&0xf)==0) printf("Clock = %d\n",(int)clock);

		switch(task)
		{
			case 0:	
																//Initial Movement
					if((clock&0xf)==0)printf("Dist = %d\n",(int)Get_Analog_Value(rc_ana_in01));
					pwm15=pwm16=170;
					pwm13=pwm14=160;
					//pwm15=pwm16=180;	//left motors
					//pwm13=pwm14=240;	//right motors
					break;
			case 1:												//Moving Forward To Get to Ball
					pwm15=pwm16=200;
					pwm13=pwm14=200;
					break;
			
			case 2:												//See the Light
					Disable_Receiver();
					pwm15=pwm16=127;			//left motors
					pwm13=pwm14=127;			//right motors
					relay5_fwd=1;
					relay5_rev=0;
					break;
			case 3:												//Hit the Ball
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					relay5_fwd=1;
					relay5_rev=0;
					break;
			case 4:												//Pull Arm Back
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					relay5_fwd=0;
					relay5_rev=0;
					break;
			case 5:												//Ram Forward Again
					relay5_fwd=0;
					relay5_rev=0;
					pwm15=pwm16=200;
					pwm13=pwm14=200;
					break;
			case 6:
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					break;
		}

		pwm15=pwm16=(unsigned char)((unsigned int)old_pwmleft+((unsigned int)25*(2000+(unsigned int)pwm15-(unsigned int)old_pwmleft)/(unsigned int)100) - ((unsigned int)20*(unsigned int)25));
		pwm13=pwm14=(unsigned char)((unsigned int)old_pwmright+((unsigned int)25*(2000+(unsigned int)pwm13-(unsigned int)old_pwmright)/(unsigned int)100) - ((unsigned int)20*(unsigned int)25));

		old_pwmleft=pwm15;
		old_pwmright=pwm13;		
		
		if(task!=0 && old_clock[task]==0) old_clock[task]=clock;

		if(!rc_dig_in01)
		{
			relay6_fwd=1;
			relay6_rev=0;	
		}
		else
		{
			relay6_fwd=0;
			relay6_rev=0;
		}

		pwm13=pwm14=50*(pwm13-127)/100+pwm13;	//slow it down for testing
		pwm15=pwm16=50*(pwm15-127)/100+pwm15;	

		Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

        Putdata(&txdata);   //even more bad things will happen if you mess with this
    }
  }
}
