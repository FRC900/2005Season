//INSIDE user_routines_fast.c
//This does not work as of yet, need to figure out why.
void User_Autonomous_Code(void)
{
  unsigned char task=0;
  unsigned int old_clock[7];
  unsigned int clock=0;
  unsigned int clock2=0;
  unsigned char old_pwmleft=127;
  unsigned char old_pwmright=127;
  unsigned char hitBall=0;
  unsigned char stripe_old=0;

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
		
		if(rc_dig_in03 && clock2>95)
		{
				if(stripe_old==0) hitBall++;
				stripe_old=1;
		}
		else
		{
			stripe_old=0;
		}
		if(hitBall>0)
		{
			if(clock<10)							//See the Light
			{
				printf("I saw the light, it was puurty\n");
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
					if(hitBall>2 && clock<100)
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
		
		if((clock%6)==0 && clock!=0) printf("Task = %d\n",(int)task);
		if((clock%6)==0 && clock!=0) printf("Clock = %d\n",(int)clock);

		switch(task)
		{
			case 0:	
																//Initial Movement
					pwm15=pwm16=162;
					pwm13=pwm14=160;
					//pwm15=pwm16=180;	//left motors
					//pwm13=pwm14=240;	//right motors
					break;
			case 1:												//Moving Forward To Get to Ball
					pwm15=pwm16=100;
					pwm13=pwm14=100;
					break;
			
			case 2:												//See the Light
					Disable_Receiver();
					pwm15=pwm16=127;			//left motors
					pwm13=pwm14=127;			//right motors
					relay5_fwd=1;
					relay5_rev=0;
					break;
			case 3:												//Hit the Ball
					pwm15=pwm16=150;		//left motors
					pwm13=pwm14=150;		//right motors
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
			case 7:
					pwm15=pwm16=100;
					pwm13=pwm14=100;
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

		clock2++;

        Putdata(&txdata);   //even more bad things will happen if you mess with this
    }
  }
}
