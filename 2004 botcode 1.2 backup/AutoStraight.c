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
		relay4_fwd=0;
		relay4_rev=1;
		if(clock<400) //if(clock>191 && clock<400) <--should look like this is wanted to wait 5 seconds (191 loops in 5 seconds) 
		{
			task=5;
			clock++;
		}
		else
		{
			task=6;
		}
		
		if((clock&0xf)==0) printf("Task = %d\n",(int)task);
		if((clock&0xf)==0) printf("Clock = %d\n",(int)clock);

		switch(task)
		{
			case 0:	
																//Initial Movement
					if((clock&0xf)==0)printf("Dist = %d\n",(int)Get_Analog_Value(rc_ana_in01));
					pwm15=pwm16=150;
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
					pwm15=pwm16=187;
					pwm13=pwm14=180;
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
