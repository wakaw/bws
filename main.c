//amp_2
//2008/12/12,15:18�X�V
//��׏d�Ɖׂƍ��E�����̃R���v���C�A���X�𓯎�����
//wata2.c���x�[�X


////////////////////////////////////////////////////////////////////////////////
//*** �w�b�_�t�@�C�� *********************************************************//
////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <string.h>												//memset()041203
#include <sys/neutrino.h>										
#include <hw/inout.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/netmgr.h>
#include <termios.h>											//tcischars()
#include <math.h>												//sin()
#include <sched.h>												//setprio()
#include <sys/types.h>											//socket
//#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <inttypes.h>		//Clock
#include <sys/syspage.h>

////////////////////////////////////////////////////////////////////////////////
//*** ������u������ ********************************************************///
////////////////////////////////////////////////////////////////////////////////
#define		PAI					3.14159
#define		HIGH_PRIO			60
#define		BUFFER_SIZE			10000
#define		BUFFER				10000
#define		MY_PULSE_CODE		_PULSE_CODE_MINAVAIL

#define		BIO_ADA				0x300							//T104-ADA
#define		BIO_C				0x340							//T104-C130
#define		BIO_SIZE			0x060

///// T104-ADA �֘A /////
#define		ADCVT				BIO_ADA+0x08
#define		ADL					BIO_ADA+0x08
#define		ADU					BIO_ADA+0x09
#define		ADSTS				BIO_ADA+0x0a

#define		DAL					BIO_ADA+0x00
#define		DAU					BIO_ADA+0x01
#define		LDAC				BIO_ADA+0x00

///// T104-C130�֘A /////
#define		Count_com				BIO_C+0x00
#define		Count_data1				BIO_C+0x01
#define		Count_data2				BIO_C+0x02
#define		Count_data3				BIO_C+0x03

#define		COUNTER_A_WRITE		0x00
#define		COUNTER_A_READ		0x04
#define		COUNTER_B_WRITE		0x02
#define		COUNTER_B_READ		0x06

#define		Reduction_Ratio		12.3
#define		Channel				2

////////////////////////////////////////////////////////////////////////////////
//*** �O���[�o���ϐ� ********************************************************///
////////////////////////////////////////////////////////////////////////////////
char recv_buf[BUFFER_SIZE];
char send_buf[BUFFER_SIZE];
FILE *fp;
char buffer[BUFFER];
char filename[BUFFER];
int count;
enum coltyp {BLACK,RED,GREEN,YELLOW,BLUE,MAGENTA,CYAN,WHITE} color;
int Samp_Cycle;
double Rec_Time;

typedef union {
	struct _pulse	pulse;
} my_message_t;
////////////////////////////////////////////////////////////////////////////////
//*** �\���� ****************************************************************///
////////////////////////////////////////////////////////////////////////////////
struct data{
	int Counter;
	int Counter_Past;
	int Abs_Counter;
	float Velocity;
	float RPS;
	float RPM;
	float Force;
	int AD_Init;
	int DA_Value1;
	int DA_Value2;
	float Target_RPM1;
	float Target_RPM2;
	float Target_Force;
	float Target_Range;
	float Input_RPM;
	float Gain;
	float Gain1;
	float Gain2;
	float Position;
	float Target_Position;
	float Compliance;
	
/*	double Pre_Load;
	double Target_Load;
	double Dev_Load;
	double Velocity;	//[pulse/sec]
	double Gain;
	double Amp_M;
	double Amp_T;
	double Freq_M;
	double Freq_T;
	double Target_Pos;
	double Real_Pulse;
	int Target_Pulse;
	int Drive_Pulse;
	int Object_Speed;
	int StartStop_Speed;
	double AD_In_P;
	double AD_Out_P;
	double AD_Out;	*/
};

struct data M[3];

////////////////////////////////////////////////////////////////////////////////
//*** �֐��̃v���g�^�C�v�錾 *************************************************//
////////////////////////////////////////////////////////////////////////////////

///// �G�X�P�[�v�V�[�P���X /////////////////////////////////////////////////////
void Beep(void);
void Cls(void);
void Clear_Line(void);
void Gotoxy(int x, int y);
void Savexy(void);
void Returnxy(void);
void Goup(int x);
void Godown(int x);
void Setcolor(int fg, int bg);
void Restore(void);
///// ���ݎ����擾�֐� /////////////////////////////////////////////////////////
void Now_Time(void);
///// ����{�[�h�C�j�V�����C�Y�֐� /////////////////////////////////////////////
void Board_Open(void);											//QNX-Only
void Board_Close(void);											//QNX-Only
///// �ᐅ���t�@�C�������֐� ///////////////////////////////////////////////////
void out2b(int a, int b, int data);								//QNX-Only
void out3b(int a, int b, int c, long data);						//QNX-Only
void out4b(int a, int b, int c, int d, long data);				//QNX-Only
int in2b(int a, int b);											//QNX-Only
int in3b(int a, int b, int c);									//QNX-Only
int in4b(int a, int b, int c, int d);							//QNX-Only
///// ���앶�����͊֐� /////////////////////////////////////////////////////////
char Yes_or_No(void);
char Char_Stdin(void);
int Int_Stdin(int a, int b);
float Float_Stdin(float a, float b);
double Double_Stdin(double a, double b);
///// �^�C�}�֘A ///////////////////////////////////////////////////////////////
void Input_Timer_Parameter(void);
void Timer1(int Samp_Cycle, double Rec_Time, void(*function)(void));			//�^�C�}�[�P�i�����Ȃ��֐��j
void Timer2(int Samp_Cycle, double Rec_Time, void(*function)(int a), int ch);	//�^�C�}�[�Q�i��������֐��j
void Infinite_Timer(int Samp_Cycle,void(*function)(void));						//QNX-Only
///// �������t�@�C�������֐� ///////////////////////////////////////////////////
void File_Open(void);
void File_Close(void);
///// �Z���T�֘A�֐� ///////////////////////////////////////////////////////////
int Count_Read(int ch);
void Count_Reset(int ch);
void Counters_Read(void);
void Counters_Reset(void);
int AD_Read(int ch);
void DA_Write(int ch, int value);
void AD_Initialize(void);
int AD_Average(int ch);
void Sensors_Display(void);
void Init_Counters(void);
///// ���j���[�֘A�֐� /////////////////////////////////////////////////////////
void Main_Menu(void);
void Display_Title(void);
///// ���䔻��֐� /////////////////////////////////////////////////////////////
void Sensors_Check(void);
void DA_Value_Check(void);
void Velocity_Check(void);
void Velocity_Display(int ch);
void Velocity_Step(void);
void Force_Const(void);
void Force_Const_2(void);
void BWS_and_Compliance(void);
//void Force_Const_3(void);
//void Step_M(void);
//void Freq_M(void);
///// �쓮 /////////////////////////////////////////////////////////////////////
void Step_Drive1(int ch);
void Step_Drive2(int ch);
void Force_Const_Drive(int ch);
void Force_Const_Drive2(int tekitou);
void BWS_and_Compliance_drive(void);
//void Freq_M_Drive(int ch);
///// �w�b�_ ///////////////////////////////////////////////////////////////////
void Step_h(int ch);
void Force_Const_h(int ch);
void Force_Const_h2(void);
void Force_Const_Log2(void);
void BWS_and_Compliance_h(void);
void BWS_and_Compliance_Log(void);
//void Step_M_h(int ch);
//void Freq_M_h(int ch);
///// ���O /////////////////////////////////////////////////////////////////////
void Step_Log1(int ch);
void Step_Log2(int ch);
void Force_Const_Log(int ch);
//void Step_M_Log(int ch);
//void Freq_M_Log(int ch);

////////////////////////////////////////////////////////////////////////////////
//*** �֐��̋L�q *************************************************************//
////////////////////////////////////////////////////////////////////////////////

///// ���C���֐� ///////////////////////////////////////////////////////////////
int main()
{
	Board_Open();
	Main_Menu();
	Board_Close();
	return(0);
}
///// ����{�[�h�C�j�V�����C�Y�֐�//////////////////////////////////////////////
void Board_Open(void)
{
	int ch;
	ThreadCtl(_NTO_TCTL_IO,0);
	mmap_device_io(BIO_SIZE,0x300);

///// �J�E���^������ /////
	out8(Count_com,0x13);		//A-2-phase x4
	out8(Count_com,0x1b);		//B-2-phase x4
	out8(Count_com,0x14);		//A-count enable set
	out8(Count_com,0x1c);		//B-count enable set
	out8(Count_com,0x16);		//separate mode
	out8(Count_com,0x0e);		//level CLR
	out8(Count_com,0x0f);		//edge CLR
	
	for(ch=1; ch<=Channel; ch++){
		M[ch].AD_Init = 2048;										//�Ƃ肠������AD�̏����l
		DA_Write(ch,2048);											//2.5V�o��
	}
}
void Board_Close(void)
{
	munmap_device_io(BIO_SIZE,0x300);
}
///// �Z���T�֘A�֐� ///////////////////////////////////////////////////////////
//*** �J�E���^�֘A *************************************************************
int Count_Read(int ch)	//�J�E���^�ǂݍ���
{
	int unsigned_data;
	int signed_data;
	switch(ch){
		case 1:
			out8(Count_com, COUNTER_A_READ);
			unsigned_data=in3b(Count_data1,Count_data2,Count_data3);
			break;
		case 2:
			out8(Count_com, COUNTER_B_READ);
			unsigned_data=in3b(Count_data1,Count_data2,Count_data3);
			break;
	}
	signed_data=(signed int)(unsigned_data<<8)>>8;
	return signed_data;
}
void Count_Reset(int ch)	//�J�E���^���Z�b�g
{
	switch(ch){
	case 1:
		out3b(Count_data1, Count_data2, Count_data3, 0x00L);
		out8(Count_com,COUNTER_A_WRITE);
		break;
	case 2:
		out3b(Count_data1, Count_data2, Count_data3, 0x00L);
		out8(Count_com,COUNTER_B_WRITE);
		break;
	}
}
void Counters_Read(void)	//�J�E���^�l�� M[ch].Counter �֑��
{
	int ch;
	for(ch=1; ch<=Channel; ch++){
		M[ch].Counter = Count_Read(ch);
	}
}
void Counters_Reset(void)	//�J�E���^�l��S�������؂�Ƀ��Z�b�g
{
	int ch;
	for(ch=1; ch<=Channel; ch++){
	Count_Reset(ch);
	}
}
//*** AD�֘A *************************************************************
int AD_Read(int ch)	//AD�ǂݍ��� �V���O���G���h�E�o�C�|�[��
{
	int data;
	out8(ADCVT,(8+(ch-1)));
	while((in8(ADSTS) & 0x80) == 0){;}
	data=((in8(ADU)<<8)+(in8(ADL)));
	if(data>=2048) data=data-4096;
	return data;
}
void DA_Write(int ch, int value) //DA�����o��
{
	out8((DAL+2*(ch-1)),value&0xff);
	out8((DAU+2*(ch-1)),value>>8);
	in8(LDAC);
}
void AD_Initialize(void)	//AD������
{
	int ch;
	char yes;
	Display_Title();										//�^�C�g���\��
	printf(" <Initialize AD Value>\n");						//�T�u�^�C�g���\��
	
	printf("\n Confirm No Load to Sensors !! -- Y or N >> ");
	yes = Yes_or_No();										//���O�擾�̗L��
	if(yes=='y'){
		for(ch=1; ch<=Channel; ch++){
			M[ch].AD_Init = AD_Average(ch);
		}
	}
	else Beep();
}
int AD_Average(int ch)	//AD1000�񕽋�
{
	int i;
	int AD1;
	int AD2 = 0;
	int data;
	
	for(i=1; i<=1000; i++){
		AD1 = AD_Read(ch);
		AD2 = AD2 + AD1;
	}
	data = AD2/1000;
	return data;
}
//*** �Z���T���� *********************************************************
void Sensors_Display(void)
{
	int ch;
	Gotoxy(0,6);
	Setcolor(GREEN,BLACK);
	Counters_Read();
	
	for(ch=1; ch<=Channel; ch++){
	printf(" ch%d Counter :%8.3f[mm] (%6d[pulse])\n",ch ,M[ch].Counter/2000.0*5.0/Reduction_Ratio ,M[ch].Counter);
	}														//�J�E���^�~�S���{
	for(ch=1; ch<=Channel; ch++){
	M[ch].Force = AD_Read(ch)/2047.0*1000.0;
	printf(" ch%d Load : %6d (%6.3f[N])\n", ch, AD_Read(ch), M[ch].Force);
	}
	printf("\n");
	Restore();
}
void Init_Counters(void)
{
	int yes;
	Sensors_Display();
	printf("\n Do you want to Reset the Counters ? -- Y or N >> ");
	yes = Yes_or_No();
	if(yes =='y'){
		Counters_Reset();
		Savexy();												//�J�[�\���ʒu�擾
		Sensors_Display();
		Returnxy();												//�J�[�\���ʒu���A
	}
}
///// ���j���[�֘A�֐� /////////////////////////////////////////////////////////
void Main_Menu(void)
{
	int Flag=1;
	int KeyInput;
	
	while(Flag){
		Cls();
		Display_Title();
		Setcolor(CYAN,BLACK);
		printf(" <Select Menu>\n\n");
		Restore();
		
		Sensors_Display();
		
		printf(" 1) Sensors Check \n");
		printf(" 2) DA Value Check \n");
		printf(" 3) Velocity Check \n");
		printf(" 4) Velocity Step Response \n");
		printf(" 5) Constant Force Control\n");
		printf(" 6) Constant Force Control(default)\n");
//		printf(" 7) Constant Force Control(range)\n");
		printf("\n");
		printf(" i) Initialize AD Value\n");
		printf("\n");
		printf(" s> Back to QNX\n\n");
		printf(" >>");
		
		KeyInput = Char_Stdin();
		switch(KeyInput){
			case '1': Sensors_Check();							break;
			case '2': DA_Value_Check();							break;
			case '3': Velocity_Check();							break;
			case '4': Velocity_Step();							break;
			case '5': Force_Const();							break;
			case '6': Force_Const_2();							break;
			case '7': BWS_and_Compliance();						break;			
			case 'i': AD_Initialize();							break;
			case 's': printf("\nBack to Qnx\n");	Flag=0;		break;
			default:printf("You Typed Invalid Command!!\n");	break;
			}
		}
}
void Display_Title(void)							//�v���O�����^�C�g���\�����֐��� 040928
{
	Cls();
	Setcolor(CYAN,BLACK);
	printf(" Weight Bearing Control Program\n");
	printf(" Copyright All Reserved by Fujie Lab. 2005\n\n");
	Restore();
}
//// �P�D�Z���T�[�̃`�F�b�N ////////////////////////////////////////////////
void Sensors_Check(void)
{
	int Flag=1;
	int yes;

	while(Flag){
		Display_Title();											//�^�C�g���\��
		printf(" <Sensors Check Menu>\n");							//�T�u�^�C�g���\��
		Sensors_Display();
		
		Input_Timer_Parameter();									//�T���v�����O�Ԋu�ƋL�^���Ԃ����
		
		Savexy();													//�J�[�\���ʒu�擾
		Beep();
		/////Timer//////////////////////////////////////////////
		Timer1(Samp_Cycle,Rec_Time,Sensors_Display);					//�^�C�}on
		
		sleep(1);
		Sensors_Display();
		Returnxy();													//�J�[�\���ʒu���A
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
//// �Q�DDA�l�̃`�F�b�N ////////////////////////////////////////////////
void DA_Value_Check(void)
{
	int Flag=1;
	int yes;
	int ch;
	double Voltage;
	int value;

	while(Flag){
		Display_Title();											//�^�C�g���\��
		printf(" <DA Value Check Menu>\n");							//�T�u�^�C�g���\��
		Sensors_Display();
		
		printf("\n Input Channel (1 or 2) >> ");					//�`�����l������
		ch = Int_Stdin(1,2);		
		printf("\n Input Voltage (0 - 5.0)[V]) >> ");				//�d������
		Voltage = Double_Stdin(0,5);
		Savexy();													//�J�[�\���ʒu�擾
		
		value = Voltage / 5.0 * 4095.0;								//value�ɕϊ�
		printf("DA Value is %d \n",value);
		DA_Write(ch,value);											//�o��
		
		sleep(1);
		Sensors_Display();
		Returnxy();													//�J�[�\���ʒu���A
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
//// �R�D���x�̃`�F�b�N ////////////////////////////////////////////////
void Velocity_Check(void)
{
	int Flag=1;
	int yes;
	int ch;
	double Voltage;
	int value;

	while(Flag){
		Display_Title();											//�^�C�g���\��
		printf(" <Velocity Check Menu>\n");							//�T�u�^�C�g���\��
		Sensors_Display();
		
		printf("\n Input Channel (1 or 2) >> ");					//�`�����l������
		ch = Int_Stdin(1,2);		
		printf("\n Input Voltage (0 - 5.0)[V]) >> ");				//�d������
		Voltage = Double_Stdin(0,5);
		Input_Timer_Parameter();									//�T���v�����O�Ԋu�ƋL�^���Ԃ����
		
		value = Voltage / 5.0 * 4095.0;								//value�ɕϊ�

		M[ch].Counter_Past = Count_Read(ch);						//�J�E���^�����l
		
		DA_Write(ch,value);											//�o��
		///////////////////////////////////////////////
		Timer2(Samp_Cycle,Rec_Time,Velocity_Display,ch);			//�^�C�}on
		DA_Write(ch,2048);											//2.5V�o��
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//�J�[�\���ʒu���A
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
void Velocity_Display(int ch) //���g�������i���[�^�j
{
	double RPS;
	M[ch].Counter = Count_Read(ch);
	RPS = (M[ch].Counter-M[ch].Counter_Past)/(2000.0*Reduction_Ratio)/(Samp_Cycle/1000.0);
	M[ch].Velocity = RPS*5.0;
	M[ch].RPM = RPS*60.0;
	Gotoxy(0,20);
	Setcolor(GREEN,BLACK);
	printf(" ch%d Velocity :%8.3f[mm/sec] %8.3f[rpm]\n",ch, M[ch].Velocity,M[ch].RPM);
	Restore();
	M[ch].Counter_Past = M[ch].Counter;
}
//// �S�D���x�X�e�b�v���� ////////////////////////////////////////////////
void Velocity_Step(void)
{
	int Flag=1;
	int yes;
	int ch;
	
	while(Flag){
		Display_Title();											//�^�C�g���\��
		printf(" <Velocity Step Response Test Menu>\n");			//�T�u�^�C�g���\��
		Sensors_Display();
		
		printf("\n Input Channel (1 or 2) >> ");					//�`�����l������
		ch = Int_Stdin(1,2);
		
		printf("\n Input RPM 1 (-830 - 830)[RPM]) >> ");			//��]������
		M[ch].Target_RPM1 = Double_Stdin(-830,830);
		M[ch].DA_Value1 = (M[ch].Target_RPM1*0.003+2.5057)/5.0*4095.0;			//value�ɕϊ�
		
		printf("\n Input RPM 2 (-830 - 830)[RPM]) >> ");			//��]������
		M[ch].Target_RPM2 = Double_Stdin(-830,830);
		M[ch].DA_Value2 = (M[ch].Target_RPM2*0.003+2.5057)/5.0*4095.0;			//value�ɕϊ�
		
		printf("\n Input Sampling Cycle (1-100)[msec] >>");
		Samp_Cycle = Int_Stdin(1,100);
		
		M[ch].Counter_Past = Count_Read(ch);						//�J�E���^�����l
		Rec_Time = 2.0;
		
		File_Open();												//�t�@�C���I�[�v��
		Step_h(ch);
		////////////////////////////////////////////////////////////////////////
		Timer2(Samp_Cycle,Rec_Time,Step_Drive1,ch);					//�^�C�}on
		Timer2(Samp_Cycle,Rec_Time,Step_Drive2,ch);					//�^�C�}on
		////////////////////////////////////////////////////////////////////////
		DA_Write(ch,2048);											//2.5V�o��
		File_Close();												//�t�@�C���N���[�Y
		
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//�J�[�\���ʒu���A
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
void Step_Drive1(int ch)
{
	double RPS;
	if(count == 0){
		printf("\n Value 1 : %d",M[ch].DA_Value1);
		DA_Write(ch,M[ch].DA_Value1);
	}
	M[ch].Counter = Count_Read(ch);
	RPS = (M[ch].Counter-M[ch].Counter_Past)/(2000.0*Reduction_Ratio)/(Samp_Cycle/1000.0);
	M[ch].Velocity = RPS * 5.0;
	M[ch].RPM = RPS * 60.0;
	Step_Log1(ch);
	M[ch].Counter_Past = M[ch].Counter;
}
void Step_Drive2(int ch)
{
	double RPS;
	if(count == 0){
		printf("\n Value 1 : %d",M[ch].DA_Value1);
		DA_Write(ch,M[ch].DA_Value2);
	}
	M[ch].Counter = Count_Read(ch);
	RPS = (M[ch].Counter-M[ch].Counter_Past)/(2000.0*Reduction_Ratio)/(Samp_Cycle/1000.0);
	M[ch].Velocity = RPS * 5.0;
	M[ch].RPM = RPS * 60.0;
	Step_Log2(ch);
	M[ch].Counter_Past = M[ch].Counter;
}
void Step_h(int ch)
{
	Now_Time();															//���ݎ����擾
	fprintf(fp,"Velocity Step Response Test(Motor)\n");					//�^�C�g���o��
	fprintf(fp,"Motor : %d\n",ch);										//ch�o��
	fprintf(fp,"%s\n",filename);										//�t�@�C�����o��
	fprintf(fp,"Sampling Cycle : %8d[msec]\n",Samp_Cycle);				//�T���v�����O�Ԋu�o��
	fprintf(fp,"Record Time : %8.3f[sec]\n\n",Rec_Time);				//�T���v�����O���ԏo��
	fprintf(fp,"Target RPM 1 : %8.3f\n",M[ch].Target_RPM1);		//�T���v�����O���ԏo��
	fprintf(fp,"Target RPM 2 : %8.3f\n\n",M[ch].Target_RPM2);		//�T���v�����O���ԏo��
	fprintf(fp,"time[msec],Target RPM,RPM\n");
}
void Step_Log1(int ch)
{
	fprintf(fp,"%8d,%8.3f,%8.3f\n",
			Samp_Cycle*count,
			M[ch].Target_RPM1,
			M[ch].RPM);
}
void Step_Log2(int ch)
{
	fprintf(fp,"%8d,%8.3f,%8.3f\n",
			Samp_Cycle*count+2000, 
			M[ch].Target_RPM2,
			M[ch].RPM);
}
//// �T�D�Ɖ׈�萧�� ////////////////////////////////////////////////
void Force_Const(void)
{
	int Flag=1;
	int yes;
	int ch;
	
	while(Flag){
		Display_Title();											//�^�C�g���\��
		printf(" <Constant Force Control Test Menu>\n");		//�T�u�^�C�g���\��
		Sensors_Display();
		
		printf("\n Input Channel (1 or 2) >> ");					//�`�����l������
		ch = Int_Stdin(1,2);		
		printf("\n Input Target Force (0 - 500)[N]) >> ");			//�Ɖחʓ���
		M[ch].Target_Force = Double_Stdin(0,500);
	
		printf("\n Input Target Range (0 - 200)[mm]) >> ");			//��]������
		M[ch].Target_Range = Double_Stdin(0,200);

		printf("\n Input Gain (UP) (0 - )[rpm/N]) >> ");					//�Q�C�����́i�̂ڂ�j
		M[ch].Gain1 = Double_Stdin(0,1000);
		
		printf("\n Input Gain (DOWN) (0 - )[rpm/N]) >> ");					//�Q�C�����́i������j
		M[ch].Gain2 = Double_Stdin(0,1000);
		
		Input_Timer_Parameter();									//�T���v�����O�Ԋu�ƋL�^���Ԃ����
		Count_Reset(ch);			
		M[ch].Counter_Past = Count_Read(ch);						//�J�E���^�����l
		
		File_Open();												//�t�@�C���I�[�v��
		Force_Const_h(ch);
		////////////////////////////////////////////////////////////////////////
		Timer2(Samp_Cycle,Rec_Time,Force_Const_Drive,ch);			//�^�C�}on
		////////////////////////////////////////////////////////////////////////
		DA_Write(ch,2048);											//2.5V�o��
		File_Close();												//�t�@�C���N���[�Y
		
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//�J�[�\���ʒu���A
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
void Force_Const_Drive(int ch)
{
	double RPS;
	double Dev_Force;
	int AD;
	int Dev_Counter;
	int MaxCounter;
	int MinCounter;
	//	int lead=5;													//�{�[���˂��̃��[�h
	//	int slit=2000;												//�G���R�[�_�̕���\
	//#define		Reduction_Ratio		12.3

	
	AD = AD_Read(ch);									//���[�h�Z�����珉���l�i�d���H�j���擾
	M[ch].Counter = Count_Read(ch);						//�J�E���^�̏����ʒu�擾
	Dev_Counter = M[ch].Counter;	//�J�E���^�̏����l����̍���
	MaxCounter = M[ch].Target_Range*2000*Reduction_Ratio/5;		//�p���X�͈̔�
	MinCounter = -M[ch].Target_Range*2000*Reduction_Ratio/5;
	
	M[ch].Force = AD/2047.0*1000.0;						//�͂̌��ݒl�擾
	Dev_Force = M[ch].Target_Force - M[ch].Force;					//�͂̕΍����v�Z
	
	if(Dev_Force > 0)	M[ch].Input_RPM = M[ch].Gain1 * Dev_Force;			//�̂ڂ艺��ŃQ�C����������
	else				M[ch].Input_RPM = M[ch].Gain2 * Dev_Force;			
	if(Dev_Counter < MinCounter) M[ch].Input_RPM = 0;
	if(MaxCounter < Dev_Counter) M[ch].Input_RPM = 0;
	if((-50<=AD)&&(AD<=50)) M[ch].Input_RPM = 0; 
	
	M[ch].DA_Value1 = (M[ch].Input_RPM*0.003+2.5057)/5.0*4095.0;	//�f�W�^���l�ɕϊ�
	if(M[ch].DA_Value1 >= 4095) M[ch].DA_Value1=4095;
	if(M[ch].DA_Value1 <= 0) M[ch].DA_Value1=0;
	DA_Write(ch,M[ch].DA_Value1);
	
	RPS = (M[ch].Counter-M[ch].Counter_Past)/(2000.0*Reduction_Ratio)/(Samp_Cycle/1000.0);
	M[ch].RPM = RPS * 60.0;

	Force_Const_Log(ch);
	M[ch].Counter_Past = MaxCounter;
}
void Force_Const_h(int ch)
{
	Now_Time();															//���ݎ����擾
	fprintf(fp,"Constant Force Control Test Menu\n");					//�^�C�g���o��
	fprintf(fp,"Motor : %d\n",ch);										//ch�o��
	fprintf(fp,"%s\n",filename);										//�t�@�C�����o��
	fprintf(fp,"Sampling Cycle : %8d[msec]\n",Samp_Cycle);				//�T���v�����O�Ԋu�o��
	fprintf(fp,"Record Time : %8.3f[sec]\n\n",Rec_Time);				//�T���v�����O���ԏo��
	fprintf(fp,"Target Force : %8.3f[N]\n",M[ch].Target_Force);
	fprintf(fp,"Gain1 : %8.3f[rpm/N]\n",M[ch].Gain1);
	fprintf(fp,"Gain2 : %8.3f[rpm/N]\n",M[ch].Gain2);
	fprintf(fp,"time[msec],Target Force,Force,InputRPM,RPM,Counter,Max Counter\n");
}
void Force_Const_Log(int ch)
{
	fprintf(fp,"%8d,%8.3f,%8.3f,%8.3f,%8.3f,%8d,%8d\n",
			Samp_Cycle*count,
			M[ch].Target_Force,
			M[ch].Force,
			M[ch].Input_RPM,
			M[ch].RPM,
			M[ch].Counter,
			M[ch].Counter_Past
			);											//change 060706
}
//// �U�D�Ɖ׈�萧��i�Q���[�^�j ////////////////////////////////////////////////
void Force_Const_2(void)
{
	int Flag=1;
	int yes;
	int ch;
	
	while(Flag){
		Display_Title();											//�^�C�g���\��
		printf(" <Constant Force Control Test Menu>\n");			//�T�u�^�C�g���\��
		Sensors_Display();
		
		printf("\n Input Target Force (0 - 500)[N]) >> ");			//��]������
		M[1].Target_Force = Double_Stdin(0,500);
		M[2].Target_Force = Double_Stdin(0,500);
		
		printf("\n Input Gain (UP) (0 - )[rpm/N]) >> ");			//�Q�C�����́i�̂ڂ�j
		M[1].Gain1 = Double_Stdin(0,1000);
		M[2].Gain1 = Double_Stdin(0,1000);
		
		printf("\n Input Gain (DOWN) (0 - )[rpm/N]) >> ");			//�Q�C�����́i������j
		M[1].Gain2 = Double_Stdin(0,1000);
		M[2].Gain2 = Double_Stdin(0,1000);
		
		Input_Timer_Parameter();									//�T���v�����O�Ԋu�ƋL�^���Ԃ����
		
		M[1].Counter_Past = Count_Read(1);						//�J�E���^�����l
		M[2].Counter_Past = Count_Read(2);
		
		File_Open();												//�t�@�C���I�[�v��
		Force_Const_h2();
		////////////////////////////////////////////////////////////////////////
		Timer2(Samp_Cycle,Rec_Time,Force_Const_Drive2,1);			//�^�C�}on
		////////////////////////////////////////////////////////////////////////
		DA_Write(1,2048);											//2.5V�o��
		DA_Write(2,2048);
		File_Close();												//�t�@�C���N���[�Y
		
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//�J�[�\���ʒu���A
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
void Force_Const_Drive2(int tekitou)
{
	double RPS1,RPS2;
	double Dev_Force1,Dev_Force2;
	int AD1,AD2;
	
	AD1 = AD_Read(1);
	AD2 = AD_Read(2);
	M[1].Counter = Count_Read(1);
	M[2].Counter = Count_Read(2);
	
	M[1].Force = AD1/2047.0*1000.0;						//�͂̌��ݒl�擾
	M[2].Force = AD2/2047.0*1000.0;
	
	Dev_Force1 = (M[1].Target_Force - (M[1].Force+M[2].Force)/2);
	Dev_Force2 = (M[2].Target_Force - (M[1].Force+M[2].Force)/2);					//�͂̕΍����v�Z
	
	if(Dev_Force1 > 0)	M[1].Input_RPM = M[1].Gain1 * Dev_Force1;			//�̂ڂ艺��ŃQ�C����������
	else				M[1].Input_RPM = M[1].Gain2 * Dev_Force1;
	if(Dev_Force2 > 0)	M[2].Input_RPM = M[2].Gain1 * Dev_Force2;			//�̂ڂ艺��ŃQ�C����������
	else				M[2].Input_RPM = M[2].Gain2 * Dev_Force2;
	
	if((-50<=AD1)&&(AD1<=50)) M[1].Input_RPM = 0; 
	if((-50<=AD2)&&(AD2<=50)) M[2].Input_RPM = 0; 
	
	M[1].DA_Value1 = (M[1].Input_RPM*0.003+2.5057)/5.0*4095.0;	//�f�W�^���l�ɕϊ�
	if(M[1].DA_Value1 >= 4095) M[1].DA_Value1=4095;
	if(M[1].DA_Value1 <= 0) M[1].DA_Value1=0;
	DA_Write(1,M[1].DA_Value1);
	
	M[2].DA_Value1 = (M[2].Input_RPM*0.003+2.5057)/5.0*4095.0;	//�f�W�^���l�ɕϊ�
	if(M[2].DA_Value1 >= 4095) M[2].DA_Value1=4095;
	if(M[2].DA_Value1 <= 0) M[2].DA_Value1=0;
	DA_Write(2,M[2].DA_Value1);

	
	RPS1 = (M[1].Counter-M[1].Counter_Past)/(2000.0*Reduction_Ratio)/(Samp_Cycle/1000.0);
	M[1].RPM = RPS1 * 60.0;
	RPS2 = (M[2].Counter-M[2].Counter_Past)/(2000.0*Reduction_Ratio)/(Samp_Cycle/1000.0);
	M[2].RPM = RPS2 * 60.0;

	Force_Const_Log2();
	M[1].Counter_Past = M[1].Counter;
	M[2].Counter_Past = M[2].Counter;
}
void Force_Const_h2(void)
{
	Now_Time();															//���ݎ����擾
	fprintf(fp,"Constant Force Control (2motor) Test Menu\n");					//�^�C�g���o��
	fprintf(fp,"%s\n",filename);										//�t�@�C�����o��
	fprintf(fp,"Sampling Cycle : %8d[msec]\n",Samp_Cycle);				//�T���v�����O�Ԋu�o��
	fprintf(fp,"Record Time : %8.3f[sec]\n\n",Rec_Time);				//�T���v�����O���ԏo��
	fprintf(fp,"Target Force : %8.3f[N]\n",M[1].Target_Force);
	fprintf(fp,"Gain1 : %8.3f[rpm/N]\n",M[1].Gain1);
	fprintf(fp,"Gain2 : %8.3f[rpm/N]\n",M[2].Gain2);
	fprintf(fp,"time[msec],Target Force,Force1,Force2,InputRPM1,InputRPM2,RPM1,RPM2\n");
}
void Force_Const_Log2(void)
{
	fprintf(fp,"%8d,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f\n",
			Samp_Cycle*count,
			M[1].Target_Force,
			M[1].Force,
			M[2].Force,
			M[1].Input_RPM,
			M[2].Input_RPM,
			M[1].RPM,
			M[2].RPM);
}
//��׏d�Ɖׂƍ��E�����̃R���v���C�A���X�𓯎�����
void BWS_and_Compliance(void)
{
	int Flag=1;
	int yes;
	int ch;
	
	while(Flag){
		Display_Title();                                            //�^�C�g���\��
		printf(" <BWS_and_Compliance_test>\n");                     //�T�u�^�C�g���\��
		Sensors_Display();
		
		printf("\n Input Target Force (BWS) (0 - 630)[N]) >> ");			//�Ɖח͓���
		M[2].Target_Force = Double_Stdin(0,630);
		
		printf("\n Input Gain (horizontal) (0 - 1000)[rpm/N]) >> ");
		M[1].Gain1 = Double_Stdin(0,1000);
		
		printf("\n Input Gain (UP) (0 - 1000)[rpm/N]) >> ");            //�Q�C�����́i�̂ڂ�(ch1)�j
		M[2].Gain1 = Double_Stdin(0,1000);
		printf("\n Input Gain (DOWN) (0 - 1000)[rpm/N]) >> ");			//�Q�C�����́i������(ch2)�j
		M[2].Gain2 = Double_Stdin(0,1000);

		printf("\n Input Target Range (horizontal) (0 - 100)[mm]) >> ");         //�Б��������
		M[1].Target_Range = Double_Stdin(0,100);
		printf("\n Input Target Range (BWS) (0 - 100)[mm]) >> ");                //�Б��������
		M[2].Target_Range = Double_Stdin(0,100);
		
		printf("\n Input Compliance (horizontal) (0.0 - 20.0)[mm/N]) >> ");      //�R���v���C�A���X�萔����
		M[1].Compliance = Double_Stdin(0,20);
		
		Input_Timer_Parameter();                                    //�T���v�����O�Ԋu�ƋL�^���Ԃ����
		
		M[1].Counter_Past = Count_Read(1);                          //�J�E���^�����l
		M[2].Counter_Past = Count_Read(2);
		
		File_Open();												//�t�@�C���I�[�v��
		BWS_and_Compliance_h();
		////////////////////////////////////////////////////////////////////////
		Timer1(Samp_Cycle,Rec_Time,BWS_and_Compliance_drive);			//�^�C�}on
		////////////////////////////////////////////////////////////////////////
		DA_Write(1,2048);											//2.5V�o��
		DA_Write(2,2048);
		File_Close();												//�t�@�C���N���[�Y
		
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//�J�[�\���ʒu���A
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
void BWS_and_Compliance_drive(void)
{
	float RPS;
	float Dev_Position,Dev_Force;
	int ch,AD1,AD2,MaxCounter1,MaxCounter2,MinCounter1,MinCounter2;
	//	int lead=5(BWS),10(Compliance);                            //�{�[���˂��̃��[�h
	//	int slit=2000;                                             //�G���R�[�_�̕���\
	//  Reduction_Ratio=12.3;

	Counters_Read();                                               //�J�E���^�擾
	AD1 = AD_Read(1);
	AD2 = AD_Read(2);                                              //���[�h�Z�����AD�ɂăf�W�^���l�擾
	M[1].Force = AD1/2047.0*1000.0;                                //�́mN�n�̌��ݒl�Ɋ��Z
	M[2].Force = AD2/2047.0*1000.0;
	
	/*��������R���v���C�A���X*/
	M[1].Target_Position = -M[1].Compliance * M[1].Force;          //�ڕW�ʒu�mmm�n�Z�o
	M[1].Position = M[1].Counter*10/2000/Reduction_Ratio;          //���݈ʒu�mmm�n�擾
	
	Dev_Position = M[1].Target_Position-M[1].Position;             //�ʒu�΍��mmm�n
	
	MaxCounter1 = M[1].Target_Range*2000*Reduction_Ratio/10;       //�p���X���
	MinCounter1 = -M[1].Target_Range*2000*Reduction_Ratio/10;      //�p���X�����@
	
	M[1].Input_RPM = M[1].Gain * Dev_Position;                     //�ʒu�t�B�[�h�o�b�N�Q�C������ڕW���x�Z�o		
	if((-0.1<=Dev_Position)&&(Dev_Position<=0.1)) M[1].Input_RPM = 0;
	if(M[1].Counter < MinCounter1) M[1].Input_RPM = 0;
	if(MaxCounter1 < M[1].Counter) M[1].Input_RPM = 0;
	
	/*���������׏d�Ɖ�*/
	
	Dev_Force = M[2].Target_Force - M[ch].Force;                   //�͂̕΍����v�Z

	MaxCounter2 = M[2].Target_Range*2000*Reduction_Ratio/5;        //�p���X�͈̔�
	MinCounter2 = -M[2].Target_Range*2000*Reduction_Ratio/5;
	
	if(Dev_Force > 0)	M[2].Input_RPM = M[1].Gain1 * Dev_Force;   //�̂ڂ艺��ŃQ�C����������
	else				M[2].Input_RPM = M[2].Gain2 * Dev_Force;			
	if(M[2].Counter < MinCounter2) M[2].Input_RPM = 0;
	if(MaxCounter2 < M[2].Counter) M[2].Input_RPM = 0;
	if((-50<=AD2)&&(AD2<=50)) M[ch].Input_RPM = 0; 
    
	/*��������*/
	for(ch=1; ch<=Channel; ch++){
		M[ch].DA_Value1 = (M[ch].Input_RPM*0.003+2.5057)/5.0*4095.0;   //�f�W�^���l�ɕϊ�
		if(M[ch].DA_Value1 >= 4095) M[ch].DA_Value1=4095;
		if(M[ch].DA_Value1 <= 0) M[ch].DA_Value1=0;
		DA_Write(ch,M[ch].DA_Value1);

		M[ch].RPS = (M[ch].Counter-M[ch].Counter_Past)/(2000.0*Reduction_Ratio)/(Samp_Cycle/1000.0);
		
	}

	M[1].Velocity = M[ch].RPS*10;
	M[2].Velocity = M[ch].RPS*5;
	BWS_and_Compliance_Log();
	M[1].Counter_Past = M[1].Counter;
	M[2].Counter_Past = M[2].Counter;
}
void BWS_and_Compliance_h(void)
{
	Now_Time();                                                         //���ݎ����擾
	fprintf(fp,"BWS_and_Compliance\n");                                 //�^�C�g���o��
	fprintf(fp,"%s\n",filename);                                        //�t�@�C�����o��
	fprintf(fp,"Sampling Cycle : %8d[msec]\n",Samp_Cycle);              //�T���v�����O�Ԋu�o��
	fprintf(fp,"Record Time : %8.3f[sec]\n\n",Rec_Time);                //�T���v�����O���ԏo��
	fprintf(fp,"Compliance(horizontal) : %8.3f[mm/N]\n",M[1].Compliance);
	fprintf(fp,"Position feedback Gain : %8.3f[rpm/mm]\n",M[1].Gain);
	fprintf(fp,"time[msec],Force(ch1)[N],Force(ch2)[N],InputRPM(ch1),InputRPM(ch2),Velocity(ch1),Velocity(ch2),Counter(ch1),Counter(ch2)\n");
}
void BWS_and_Compliance_Log(void)
{
	fprintf(fp,"%8d,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8d,%8d\n",
			Samp_Cycle*count,
			M[1].Force,
			M[2].Force,
			M[1].Input_RPM,
			M[2].Input_RPM,
			M[1].Velocity,
			M[2].Velocity,
			M[1].Counter,
			M[2].Counter
			);				
}
/*
//// �P�D�X�e�b�v�����i���[�^�j ////////////////////////////////////////////////
void Step_M(void)
{
	int Flag=1;
	int yes;
	int ch;

	while(Flag){
		Display_Title();										//�^�C�g���\��
		printf(" <Step Response Test (Motor) Menu>\n");			//�T�u�^�C�g���\��
		Counters_Open();
		Counters_Reset();
		Sensors_Display();
		
		printf(" Input Channel (1-2) >> ");							//�@�`�����l������
		ch = Int_Stdin(1,2);
		
		printf("\n Input Target_Pos (0 - 25)[mm]) >> ");			//�A�ڕW��������
		M[ch].Target_Pos = Double_Stdin(-25,25);
		M[ch].Drive_Pulse = (1000 * M[ch].Target_Pos - M[ch].Abs_External);	//��Βl���͂ɕύX

		printf("\n Do you want to record the Encorder Log ? -- Y or N >> ");
		yes = Yes_or_No();											//���O�擾�̗L��
		
		if(yes=='y'){
			File_Open();											//�t�@�C���I�[�v��
			Input_Timer_Parameter(ch);								//�T���v�����O�Ԋu�ƋL�^���Ԃ����
			Step_M_h(ch);											//���O�̃w�b�_�o��
			Savexy();												//�J�[�\���ʒu�擾
			Beep();
			Drive(M[ch].Drive_Pulse,ch);							//�쓮
			/////Timer//////////////////////////////////////////////
			Timer(M[ch].Samp_Cycle,M[ch].Rec_Time,Step_M_Log,ch);	//�^�C�}on
			Drive_End();											//�쓮�I���m�F
			
			File_Close();											//�t�@�C���N���[�Y
		}
		else{
			Savexy();												//�J�[�\���ʒu�擾
			Beep();													//�r�[�v
			Drive(M[ch].Drive_Pulse,ch);							//�쓮
			Drive_End();											//�쓮�I���m�F
		}
		sleep(1);
		Counters_Close();
		Sensors_Display();
		Returnxy();												//�J�[�\���ʒu���A
		Godown(2);
		printf("\n Now Drive is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
//// �w�b�_ ////
void Step_M_h(int ch)
{
	Now_Time();															//���ݎ����擾
	fprintf(fp,"Step Response Test (Motor)\n");							//�^�C�g���o��
	fprintf(fp,"Motor : %d\n",ch);										//ch�o��
	fprintf(fp,"%s\n",filename);										//�t�@�C�����o��
	fprintf(fp,"Target_Position : %8.3f[mm]\n",M[ch].Target_Pos);		//�ڕW�����o��
	fprintf(fp,"Basic_Postion : %8.3f[mm]\n",M[ch].Abs_External/1000.0);
	fprintf(fp,"Sampling Cycle : %d[msec]\n",M[ch].Samp_Cycle);			//�T���v�����O�Ԋu�o��
	fprintf(fp,"Record Time : %8.3f[msec]\n\n",M[ch].Rec_Time);			//�T���v�����O���ԏo��
	fprintf(fp,"time[msec],Internal,Abs\n");
}
//// ���O ////
void Step_M_Log(int ch)
{
	Counters_Read();													//�J�E���^�l��������
	fprintf(fp,"%8d,%8d,%8d\n",
			M[ch].Samp_Cycle*count, 									//�o�ߎ���
			M[ch].Internal+M[ch].Abs_External, 							//�����J�E���^
			M[ch].External+M[ch].Abs_External);							//�O���J�E���^��Βl
}
*/
////////////////////////////////////////////////////////////////////////////////
///// ���̑��ׂ����֐� /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///// �G�X�P�[�v�V�[�P���X /////////////////////////////////////////////////////
void Beep(void)													//�r�[�v��
{
	printf("\a");
}
void Cls(void)													//��ʏ���
{
    printf("\x1b[2J");
}
void Clear_Line(void)
{
	printf("\x1b[K");
}
void Gotoxy(int x, int y)										//�J�[�\����Έʒu�ړ�
{
    printf("\x1b[%d;%dH", y, x);
}
void Savexy(void)
{
	printf("\x1b[s");
}
void Returnxy(void)
{
	printf("\x1b[u");
}
void Goup(int x)												//�J�[�\�����Έʒu�ړ��i��j
{
	printf("\x1b[%dA", x);
}
void Godown(int x)												//�J�[�\�����Έʒu�ړ��i���j
{
	printf("\x1b[%dB", x);
}
void Setcolor(int fg, int bg)									//�\���F�ݒ�
{
    printf("\x1b[1;%d;%dm", fg + 30, bg + 40);
}
void Restore(void)												//�����l�ɕ���
{
    printf("\x1b[0;37;40m");
}
///// ���ݎ����擾�֐� /////////////////////////////////////////////////////////
void Now_Time(void)
{
	time_t jikoku;
	struct tm *lt;
	char *p;
	time(&jikoku);
	lt=localtime(&jikoku);
	p=asctime(lt);
	fprintf(fp,"%s\n",p);
}
///// ���앶�����͊֐� /////////////////////////////////////////////////////////
char Yes_or_No(void)
{
	char DataInput;
	Savexy();
	while(1){
		rewind(stdin);
		fgets(buffer,BUFFER,stdin);
		sscanf(buffer,"%c",&DataInput);
		if((DataInput=='y') || (DataInput=='n'))	break;
		Returnxy();
		Clear_Line();
	}
	Returnxy();
	Clear_Line();
	printf("%c\n",DataInput);
	return DataInput;
}
char Char_Stdin(void)
{
	char DataInput;
	Savexy();
	rewind(stdin);
	fgets(buffer,BUFFER,stdin);
	sscanf(buffer,"%c",&DataInput);
	Returnxy();
	Clear_Line();
	printf("%c\n",DataInput);
	
	return DataInput;
}
int Int_Stdin(int a, int b)										//stdin�����int�^�ǂݍ��݁@040928
{
	int DataInput;
	Savexy();
	while(1){
		rewind(stdin);
		fgets(buffer,BUFFER,stdin);
		sscanf(buffer,"%d",&DataInput);
		if((a<=DataInput) && (DataInput<=b))	break;
		Returnxy();
		Clear_Line();
	}
	Returnxy();
	Clear_Line();
	printf("%d\n",DataInput);
	return DataInput;
}
float Float_Stdin(float a, float b)									//stdin�����float�^�ǂݍ��݁@040928
{
	float DataInput;
	Savexy();
	while(1){
		rewind(stdin);
		fgets(buffer,BUFFER,stdin);
		sscanf(buffer,"%f",&DataInput);
		if((a<=DataInput) && (DataInput<=b))	break;
		Returnxy();
		Clear_Line();
	}
	Returnxy();
	Clear_Line();
	printf("%.3f\n",DataInput);
	return DataInput;
}
double Double_Stdin(double a, double b)								//stdin�����Double�^�ǂݍ��݁@040929
{
	double DataInput;
	Savexy();
	while(1){
		rewind(stdin);
		fgets(buffer,BUFFER,stdin);
		sscanf(buffer,"%lf",&DataInput);
		if((a<=DataInput) && (DataInput<=b))	break;
		Returnxy();
		Clear_Line();
	}
	Returnxy();
	printf("%.3f\n",DataInput);
	return DataInput;
}

///// �ᐅ���t�@�C�������֐� ///////////////////////////////////////////////////
void out2b(int a, int b, int data)								//2byte data out
{
	out8(a,data>>8);
	out8(b,data);
}
void out3b(int a, int b, int c, long data)						//3byte data out 
{
	out8(a,data>>16);
	out8(b,data>>8);
	out8(c,data);
}
void out4b(int a, int b, int c, int d, long data)				//4byte data out
{
	out8(a,data>>24);
	out8(b,data>>16);
	out8(c,data>>8);
	out8(d,data);
}
int in2b(int a, int b)
{
	int d1,d2,d3;
	d1=in8(a);
	d2=in8(b);
	d3 = d1<<8 | d2;
	return d3;
}
int in3b(int a, int b, int c)
{
	int d1,d2,d3,d4;
	d1=in8(a);
	d2=in8(b);
	d3=in8(c);
	d4 = d1<<16 | d2<<8 | d3;
	return d4;
}
int in4b(int a, int b, int c, int d)
{
	int d1,d2,d3,d4,d5;
	d1=in8(a);
	d2=in8(b);
	d3=in8(c);
	d4=in8(d);
	d5 = d1<<24 | d2<<16 | d3<<8 | d4;
	return d5;
}
///// �������t�@�C�������֐� ///////////////////////////////////////////////////
void File_Open(void)
{
	printf("\n File Name (.csv) >> ");
	gets(filename);												//�t�@�C��������
	if((fp=fopen(filename,"a"))==NULL){							//�������݃��[�h
		printf(" File Open Failed !\n");
		exit(1);
		}
}
void File_Close(void)
{
	fclose(fp);
}
///// �^�C�}�p�����[�^���͊֐� /////////////////////////////////////////////////
void Input_Timer_Parameter(void)
{
	printf("\n Input Sampling Cycle (1-100)[msec] >>");
	Samp_Cycle = Int_Stdin(1,100);
	printf("\n Input Record Time (1-100)[sec] >> ");
	Rec_Time = Double_Stdin(1,100);
}
///// �L���^�C�} ///////////////////////////////////////////////////////////////
void Timer1(int Samp_Cycle, double Rec_Time, void(*function)(void))
{
	struct sigevent event;
	struct itimerspec itime;
	timer_t timer_id;
	int chid;
	int rcvid;
	my_message_t msg;
	uint64_t CPS, cycle1, cycle2, ncycles;						//���Ԍv���֘A
	float sec;

	setprio(0, HIGH_PRIO);
	
	chid = ChannelCreate(0);
	
	event.sigev_notify = SIGEV_PULSE;
	event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
	event.sigev_priority = getprio(0);
	event.sigev_code = MY_PULSE_CODE;
	
	timer_create(CLOCK_REALTIME, &event, &timer_id);
	
	itime.it_value.tv_sec = 0;
	itime.it_value.tv_nsec = Samp_Cycle * 1000000;		//msec
	itime.it_interval.tv_sec = 0;
	itime.it_interval.tv_nsec = Samp_Cycle * 1000000;	//msec
	timer_settime(timer_id, 0, &itime, NULL);
	
	/* snap the time */
	cycle1=ClockCycles();
	
	for(count=0; count<=(Rec_Time*1000/Samp_Cycle); count++) {
		rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
		if (rcvid == 0) {
			if (msg.pulse.code == MY_PULSE_CODE) {
				function();
			}
		}
	}
	/* snap the time again */
	cycle2=ClockCycles( );
	ncycles=cycle2-cycle1;
	//printf("\n %lld Cycles Elapsed \n", ncycles);
	
	/* find out how many cycles per second */
	CPS = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
	//printf("\n This system has %lld cycles/sec.\n",CPS);
	sec=(float)ncycles/CPS;
	printf("\n The cycles in seconds is %f \n",sec);
}
void Timer2(int Samp_Cycle, double Rec_Time, void(*function)(int a), int ch)
{
	struct sigevent event;
	struct itimerspec itime;
	timer_t timer_id;
	int chid;
	int rcvid;
	my_message_t msg;
	uint64_t CPS, cycle1, cycle2, ncycles;						//���Ԍv���֘A
	float sec;

	setprio(0, HIGH_PRIO);
	
	chid = ChannelCreate(0);
	
	event.sigev_notify = SIGEV_PULSE;
	event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
	event.sigev_priority = getprio(0);
	event.sigev_code = MY_PULSE_CODE;
	
	timer_create(CLOCK_REALTIME, &event, &timer_id);
	
	itime.it_value.tv_sec = 0;
	itime.it_value.tv_nsec = Samp_Cycle * 1000000;		//msec
	itime.it_interval.tv_sec = 0;
	itime.it_interval.tv_nsec = Samp_Cycle * 1000000;	//msec
	timer_settime(timer_id, 0, &itime, NULL);
	
	/* snap the time */
	cycle1=ClockCycles();
	
	for(count=0; count<=(Rec_Time*1000/Samp_Cycle); count++) {
		rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
		if (rcvid == 0) {
			if (msg.pulse.code == MY_PULSE_CODE) {
				function(ch);
			}
		}
	}
	/* snap the time again */
	cycle2=ClockCycles( );
	ncycles=cycle2-cycle1;
	//printf("\n %lld Cycles Elapsed \n", ncycles);
	
	/* find out how many cycles per second */
	CPS = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
	//printf("\n This system has %lld cycles/sec.\n",CPS);
	sec=(float)ncycles/CPS;
	printf("\n The cycles in seconds is %f \n",sec);
}
/*
///// �����^�C�} ///////////////////////////////////////////////////////////////
void Infinite_Timer(int Samp_Cycle,void(*function)(void))
{
	struct sigevent event;
	struct itimerspec itime;
	timer_t timer_id;
	int chid;
	int rcvid;
	my_message_t msg;
	int filedes = 0;

	setprio(0, HIGH_PRIO);
	
	chid = ChannelCreate(0);
	
	event.sigev_notify = SIGEV_PULSE;
	event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
	event.sigev_priority = getprio(0);
	event.sigev_code = MY_PULSE_CODE;
	
	timer_create(CLOCK_REALTIME, &event, &timer_id);
	
	itime.it_value.tv_sec = 0;
	itime.it_value.tv_nsec = Samp_Cycle * 1000000;		//msec
	itime.it_interval.tv_sec = 0;
	itime.it_interval.tv_nsec = Samp_Cycle * 1000000;	//msec
	timer_settime(timer_id, 0, &itime, NULL);
	
	while(!tcischars(filedes)) {
		rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
		if (rcvid == 0) {
			if (msg.pulse.code == MY_PULSE_CODE) {
				function();
				Check_AD_Log();				//���ƂŏC���I�I�I�I�I�I
			}
		}
	}
}
*/