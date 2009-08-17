//amp_2
//2008/12/12,15:18更新
//定荷重免荷と左右方向のコンプライアンスを同時制御
//wata2.cをベース


////////////////////////////////////////////////////////////////////////////////
//*** ヘッダファイル *********************************************************//
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
//*** 文字列置き換え ********************************************************///
////////////////////////////////////////////////////////////////////////////////
#define		PAI					3.14159
#define		HIGH_PRIO			60
#define		BUFFER_SIZE			10000
#define		BUFFER				10000
#define		MY_PULSE_CODE		_PULSE_CODE_MINAVAIL

#define		BIO_ADA				0x300							//T104-ADA
#define		BIO_C				0x340							//T104-C130
#define		BIO_SIZE			0x060

///// T104-ADA 関連 /////
#define		ADCVT				BIO_ADA+0x08
#define		ADL					BIO_ADA+0x08
#define		ADU					BIO_ADA+0x09
#define		ADSTS				BIO_ADA+0x0a

#define		DAL					BIO_ADA+0x00
#define		DAU					BIO_ADA+0x01
#define		LDAC				BIO_ADA+0x00

///// T104-C130関連 /////
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
//*** グローバル変数 ********************************************************///
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
//*** 構造体 ****************************************************************///
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
//*** 関数のプロトタイプ宣言 *************************************************//
////////////////////////////////////////////////////////////////////////////////

///// エスケープシーケンス /////////////////////////////////////////////////////
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
///// 現在時刻取得関数 /////////////////////////////////////////////////////////
void Now_Time(void);
///// 制御ボードイニシャライズ関数 /////////////////////////////////////////////
void Board_Open(void);											//QNX-Only
void Board_Close(void);											//QNX-Only
///// 低水準ファイル処理関数 ///////////////////////////////////////////////////
void out2b(int a, int b, int data);								//QNX-Only
void out3b(int a, int b, int c, long data);						//QNX-Only
void out4b(int a, int b, int c, int d, long data);				//QNX-Only
int in2b(int a, int b);											//QNX-Only
int in3b(int a, int b, int c);									//QNX-Only
int in4b(int a, int b, int c, int d);							//QNX-Only
///// 自作文字入力関数 /////////////////////////////////////////////////////////
char Yes_or_No(void);
char Char_Stdin(void);
int Int_Stdin(int a, int b);
float Float_Stdin(float a, float b);
double Double_Stdin(double a, double b);
///// タイマ関連 ///////////////////////////////////////////////////////////////
void Input_Timer_Parameter(void);
void Timer1(int Samp_Cycle, double Rec_Time, void(*function)(void));			//タイマー１（引数なし関数）
void Timer2(int Samp_Cycle, double Rec_Time, void(*function)(int a), int ch);	//タイマー２（引数あり関数）
void Infinite_Timer(int Samp_Cycle,void(*function)(void));						//QNX-Only
///// 高水準ファイル処理関数 ///////////////////////////////////////////////////
void File_Open(void);
void File_Close(void);
///// センサ関連関数 ///////////////////////////////////////////////////////////
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
///// メニュー関連関数 /////////////////////////////////////////////////////////
void Main_Menu(void);
void Display_Title(void);
///// 制御判定関数 /////////////////////////////////////////////////////////////
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
///// 駆動 /////////////////////////////////////////////////////////////////////
void Step_Drive1(int ch);
void Step_Drive2(int ch);
void Force_Const_Drive(int ch);
void Force_Const_Drive2(int tekitou);
void BWS_and_Compliance_drive(void);
//void Freq_M_Drive(int ch);
///// ヘッダ ///////////////////////////////////////////////////////////////////
void Step_h(int ch);
void Force_Const_h(int ch);
void Force_Const_h2(void);
void Force_Const_Log2(void);
void BWS_and_Compliance_h(void);
void BWS_and_Compliance_Log(void);
//void Step_M_h(int ch);
//void Freq_M_h(int ch);
///// ログ /////////////////////////////////////////////////////////////////////
void Step_Log1(int ch);
void Step_Log2(int ch);
void Force_Const_Log(int ch);
//void Step_M_Log(int ch);
//void Freq_M_Log(int ch);

////////////////////////////////////////////////////////////////////////////////
//*** 関数の記述 *************************************************************//
////////////////////////////////////////////////////////////////////////////////

///// メイン関数 ///////////////////////////////////////////////////////////////
int main()
{
	Board_Open();
	Main_Menu();
	Board_Close();
	return(0);
}
///// 制御ボードイニシャライズ関数//////////////////////////////////////////////
void Board_Open(void)
{
	int ch;
	ThreadCtl(_NTO_TCTL_IO,0);
	mmap_device_io(BIO_SIZE,0x300);

///// カウンタ初期化 /////
	out8(Count_com,0x13);		//A-2-phase x4
	out8(Count_com,0x1b);		//B-2-phase x4
	out8(Count_com,0x14);		//A-count enable set
	out8(Count_com,0x1c);		//B-count enable set
	out8(Count_com,0x16);		//separate mode
	out8(Count_com,0x0e);		//level CLR
	out8(Count_com,0x0f);		//edge CLR
	
	for(ch=1; ch<=Channel; ch++){
		M[ch].AD_Init = 2048;										//とりあえずのADの初期値
		DA_Write(ch,2048);											//2.5V出力
	}
}
void Board_Close(void)
{
	munmap_device_io(BIO_SIZE,0x300);
}
///// センサ関連関数 ///////////////////////////////////////////////////////////
//*** カウンタ関連 *************************************************************
int Count_Read(int ch)	//カウンタ読み込み
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
void Count_Reset(int ch)	//カウンタリセット
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
void Counters_Read(void)	//カウンタ値を M[ch].Counter へ代入
{
	int ch;
	for(ch=1; ch<=Channel; ch++){
		M[ch].Counter = Count_Read(ch);
	}
}
void Counters_Reset(void)	//カウンタ値を全部いっぺんにリセット
{
	int ch;
	for(ch=1; ch<=Channel; ch++){
	Count_Reset(ch);
	}
}
//*** AD関連 *************************************************************
int AD_Read(int ch)	//AD読み込み シングルエンド・バイポーラ
{
	int data;
	out8(ADCVT,(8+(ch-1)));
	while((in8(ADSTS) & 0x80) == 0){;}
	data=((in8(ADU)<<8)+(in8(ADL)));
	if(data>=2048) data=data-4096;
	return data;
}
void DA_Write(int ch, int value) //DA書き出し
{
	out8((DAL+2*(ch-1)),value&0xff);
	out8((DAU+2*(ch-1)),value>>8);
	in8(LDAC);
}
void AD_Initialize(void)	//AD初期化
{
	int ch;
	char yes;
	Display_Title();										//タイトル表示
	printf(" <Initialize AD Value>\n");						//サブタイトル表示
	
	printf("\n Confirm No Load to Sensors !! -- Y or N >> ");
	yes = Yes_or_No();										//ログ取得の有無
	if(yes=='y'){
		for(ch=1; ch<=Channel; ch++){
			M[ch].AD_Init = AD_Average(ch);
		}
	}
	else Beep();
}
int AD_Average(int ch)	//AD1000回平均
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
//*** センサ総合 *********************************************************
void Sensors_Display(void)
{
	int ch;
	Gotoxy(0,6);
	Setcolor(GREEN,BLACK);
	Counters_Read();
	
	for(ch=1; ch<=Channel; ch++){
	printf(" ch%d Counter :%8.3f[mm] (%6d[pulse])\n",ch ,M[ch].Counter/2000.0*5.0/Reduction_Ratio ,M[ch].Counter);
	}														//カウンタ×４逓倍
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
		Savexy();												//カーソル位置取得
		Sensors_Display();
		Returnxy();												//カーソル位置復帰
	}
}
///// メニュー関連関数 /////////////////////////////////////////////////////////
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
void Display_Title(void)							//プログラムタイトル表示を関数化 040928
{
	Cls();
	Setcolor(CYAN,BLACK);
	printf(" Weight Bearing Control Program\n");
	printf(" Copyright All Reserved by Fujie Lab. 2005\n\n");
	Restore();
}
//// １．センサーのチェック ////////////////////////////////////////////////
void Sensors_Check(void)
{
	int Flag=1;
	int yes;

	while(Flag){
		Display_Title();											//タイトル表示
		printf(" <Sensors Check Menu>\n");							//サブタイトル表示
		Sensors_Display();
		
		Input_Timer_Parameter();									//サンプリング間隔と記録時間を入力
		
		Savexy();													//カーソル位置取得
		Beep();
		/////Timer//////////////////////////////////////////////
		Timer1(Samp_Cycle,Rec_Time,Sensors_Display);					//タイマon
		
		sleep(1);
		Sensors_Display();
		Returnxy();													//カーソル位置復帰
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
//// ２．DA値のチェック ////////////////////////////////////////////////
void DA_Value_Check(void)
{
	int Flag=1;
	int yes;
	int ch;
	double Voltage;
	int value;

	while(Flag){
		Display_Title();											//タイトル表示
		printf(" <DA Value Check Menu>\n");							//サブタイトル表示
		Sensors_Display();
		
		printf("\n Input Channel (1 or 2) >> ");					//チャンネル入力
		ch = Int_Stdin(1,2);		
		printf("\n Input Voltage (0 - 5.0)[V]) >> ");				//電圧入力
		Voltage = Double_Stdin(0,5);
		Savexy();													//カーソル位置取得
		
		value = Voltage / 5.0 * 4095.0;								//valueに変換
		printf("DA Value is %d \n",value);
		DA_Write(ch,value);											//出力
		
		sleep(1);
		Sensors_Display();
		Returnxy();													//カーソル位置復帰
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
//// ３．速度のチェック ////////////////////////////////////////////////
void Velocity_Check(void)
{
	int Flag=1;
	int yes;
	int ch;
	double Voltage;
	int value;

	while(Flag){
		Display_Title();											//タイトル表示
		printf(" <Velocity Check Menu>\n");							//サブタイトル表示
		Sensors_Display();
		
		printf("\n Input Channel (1 or 2) >> ");					//チャンネル入力
		ch = Int_Stdin(1,2);		
		printf("\n Input Voltage (0 - 5.0)[V]) >> ");				//電圧入力
		Voltage = Double_Stdin(0,5);
		Input_Timer_Parameter();									//サンプリング間隔と記録時間を入力
		
		value = Voltage / 5.0 * 4095.0;								//valueに変換

		M[ch].Counter_Past = Count_Read(ch);						//カウンタ初期値
		
		DA_Write(ch,value);											//出力
		///////////////////////////////////////////////
		Timer2(Samp_Cycle,Rec_Time,Velocity_Display,ch);			//タイマon
		DA_Write(ch,2048);											//2.5V出力
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//カーソル位置復帰
		printf("\n Now Check is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
void Velocity_Display(int ch) //周波数応答（モータ）
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
//// ４．速度ステップ入力 ////////////////////////////////////////////////
void Velocity_Step(void)
{
	int Flag=1;
	int yes;
	int ch;
	
	while(Flag){
		Display_Title();											//タイトル表示
		printf(" <Velocity Step Response Test Menu>\n");			//サブタイトル表示
		Sensors_Display();
		
		printf("\n Input Channel (1 or 2) >> ");					//チャンネル入力
		ch = Int_Stdin(1,2);
		
		printf("\n Input RPM 1 (-830 - 830)[RPM]) >> ");			//回転数入力
		M[ch].Target_RPM1 = Double_Stdin(-830,830);
		M[ch].DA_Value1 = (M[ch].Target_RPM1*0.003+2.5057)/5.0*4095.0;			//valueに変換
		
		printf("\n Input RPM 2 (-830 - 830)[RPM]) >> ");			//回転数入力
		M[ch].Target_RPM2 = Double_Stdin(-830,830);
		M[ch].DA_Value2 = (M[ch].Target_RPM2*0.003+2.5057)/5.0*4095.0;			//valueに変換
		
		printf("\n Input Sampling Cycle (1-100)[msec] >>");
		Samp_Cycle = Int_Stdin(1,100);
		
		M[ch].Counter_Past = Count_Read(ch);						//カウンタ初期値
		Rec_Time = 2.0;
		
		File_Open();												//ファイルオープン
		Step_h(ch);
		////////////////////////////////////////////////////////////////////////
		Timer2(Samp_Cycle,Rec_Time,Step_Drive1,ch);					//タイマon
		Timer2(Samp_Cycle,Rec_Time,Step_Drive2,ch);					//タイマon
		////////////////////////////////////////////////////////////////////////
		DA_Write(ch,2048);											//2.5V出力
		File_Close();												//ファイルクローズ
		
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//カーソル位置復帰
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
	Now_Time();															//現在時刻取得
	fprintf(fp,"Velocity Step Response Test(Motor)\n");					//タイトル出力
	fprintf(fp,"Motor : %d\n",ch);										//ch出力
	fprintf(fp,"%s\n",filename);										//ファイル名出力
	fprintf(fp,"Sampling Cycle : %8d[msec]\n",Samp_Cycle);				//サンプリング間隔出力
	fprintf(fp,"Record Time : %8.3f[sec]\n\n",Rec_Time);				//サンプリング時間出力
	fprintf(fp,"Target RPM 1 : %8.3f\n",M[ch].Target_RPM1);		//サンプリング時間出力
	fprintf(fp,"Target RPM 2 : %8.3f\n\n",M[ch].Target_RPM2);		//サンプリング時間出力
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
//// ５．免荷一定制御 ////////////////////////////////////////////////
void Force_Const(void)
{
	int Flag=1;
	int yes;
	int ch;
	
	while(Flag){
		Display_Title();											//タイトル表示
		printf(" <Constant Force Control Test Menu>\n");		//サブタイトル表示
		Sensors_Display();
		
		printf("\n Input Channel (1 or 2) >> ");					//チャンネル入力
		ch = Int_Stdin(1,2);		
		printf("\n Input Target Force (0 - 500)[N]) >> ");			//免荷量入力
		M[ch].Target_Force = Double_Stdin(0,500);
	
		printf("\n Input Target Range (0 - 200)[mm]) >> ");			//回転数入力
		M[ch].Target_Range = Double_Stdin(0,200);

		printf("\n Input Gain (UP) (0 - )[rpm/N]) >> ");					//ゲイン入力（のぼり）
		M[ch].Gain1 = Double_Stdin(0,1000);
		
		printf("\n Input Gain (DOWN) (0 - )[rpm/N]) >> ");					//ゲイン入力（くだり）
		M[ch].Gain2 = Double_Stdin(0,1000);
		
		Input_Timer_Parameter();									//サンプリング間隔と記録時間を入力
		Count_Reset(ch);			
		M[ch].Counter_Past = Count_Read(ch);						//カウンタ初期値
		
		File_Open();												//ファイルオープン
		Force_Const_h(ch);
		////////////////////////////////////////////////////////////////////////
		Timer2(Samp_Cycle,Rec_Time,Force_Const_Drive,ch);			//タイマon
		////////////////////////////////////////////////////////////////////////
		DA_Write(ch,2048);											//2.5V出力
		File_Close();												//ファイルクローズ
		
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//カーソル位置復帰
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
	//	int lead=5;													//ボールねじのリード
	//	int slit=2000;												//エンコーダの分解能
	//#define		Reduction_Ratio		12.3

	
	AD = AD_Read(ch);									//ロードセルから初期値（電圧？）を取得
	M[ch].Counter = Count_Read(ch);						//カウンタの初期位置取得
	Dev_Counter = M[ch].Counter;	//カウンタの初期値からの差分
	MaxCounter = M[ch].Target_Range*2000*Reduction_Ratio/5;		//パルスの範囲
	MinCounter = -M[ch].Target_Range*2000*Reduction_Ratio/5;
	
	M[ch].Force = AD/2047.0*1000.0;						//力の現在値取得
	Dev_Force = M[ch].Target_Force - M[ch].Force;					//力の偏差を計算
	
	if(Dev_Force > 0)	M[ch].Input_RPM = M[ch].Gain1 * Dev_Force;			//のぼり下りでゲインがちがう
	else				M[ch].Input_RPM = M[ch].Gain2 * Dev_Force;			
	if(Dev_Counter < MinCounter) M[ch].Input_RPM = 0;
	if(MaxCounter < Dev_Counter) M[ch].Input_RPM = 0;
	if((-50<=AD)&&(AD<=50)) M[ch].Input_RPM = 0; 
	
	M[ch].DA_Value1 = (M[ch].Input_RPM*0.003+2.5057)/5.0*4095.0;	//デジタル値に変換
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
	Now_Time();															//現在時刻取得
	fprintf(fp,"Constant Force Control Test Menu\n");					//タイトル出力
	fprintf(fp,"Motor : %d\n",ch);										//ch出力
	fprintf(fp,"%s\n",filename);										//ファイル名出力
	fprintf(fp,"Sampling Cycle : %8d[msec]\n",Samp_Cycle);				//サンプリング間隔出力
	fprintf(fp,"Record Time : %8.3f[sec]\n\n",Rec_Time);				//サンプリング時間出力
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
//// ６．免荷一定制御（２モータ） ////////////////////////////////////////////////
void Force_Const_2(void)
{
	int Flag=1;
	int yes;
	int ch;
	
	while(Flag){
		Display_Title();											//タイトル表示
		printf(" <Constant Force Control Test Menu>\n");			//サブタイトル表示
		Sensors_Display();
		
		printf("\n Input Target Force (0 - 500)[N]) >> ");			//回転数入力
		M[1].Target_Force = Double_Stdin(0,500);
		M[2].Target_Force = Double_Stdin(0,500);
		
		printf("\n Input Gain (UP) (0 - )[rpm/N]) >> ");			//ゲイン入力（のぼり）
		M[1].Gain1 = Double_Stdin(0,1000);
		M[2].Gain1 = Double_Stdin(0,1000);
		
		printf("\n Input Gain (DOWN) (0 - )[rpm/N]) >> ");			//ゲイン入力（くだり）
		M[1].Gain2 = Double_Stdin(0,1000);
		M[2].Gain2 = Double_Stdin(0,1000);
		
		Input_Timer_Parameter();									//サンプリング間隔と記録時間を入力
		
		M[1].Counter_Past = Count_Read(1);						//カウンタ初期値
		M[2].Counter_Past = Count_Read(2);
		
		File_Open();												//ファイルオープン
		Force_Const_h2();
		////////////////////////////////////////////////////////////////////////
		Timer2(Samp_Cycle,Rec_Time,Force_Const_Drive2,1);			//タイマon
		////////////////////////////////////////////////////////////////////////
		DA_Write(1,2048);											//2.5V出力
		DA_Write(2,2048);
		File_Close();												//ファイルクローズ
		
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//カーソル位置復帰
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
	
	M[1].Force = AD1/2047.0*1000.0;						//力の現在値取得
	M[2].Force = AD2/2047.0*1000.0;
	
	Dev_Force1 = (M[1].Target_Force - (M[1].Force+M[2].Force)/2);
	Dev_Force2 = (M[2].Target_Force - (M[1].Force+M[2].Force)/2);					//力の偏差を計算
	
	if(Dev_Force1 > 0)	M[1].Input_RPM = M[1].Gain1 * Dev_Force1;			//のぼり下りでゲインがちがう
	else				M[1].Input_RPM = M[1].Gain2 * Dev_Force1;
	if(Dev_Force2 > 0)	M[2].Input_RPM = M[2].Gain1 * Dev_Force2;			//のぼり下りでゲインがちがう
	else				M[2].Input_RPM = M[2].Gain2 * Dev_Force2;
	
	if((-50<=AD1)&&(AD1<=50)) M[1].Input_RPM = 0; 
	if((-50<=AD2)&&(AD2<=50)) M[2].Input_RPM = 0; 
	
	M[1].DA_Value1 = (M[1].Input_RPM*0.003+2.5057)/5.0*4095.0;	//デジタル値に変換
	if(M[1].DA_Value1 >= 4095) M[1].DA_Value1=4095;
	if(M[1].DA_Value1 <= 0) M[1].DA_Value1=0;
	DA_Write(1,M[1].DA_Value1);
	
	M[2].DA_Value1 = (M[2].Input_RPM*0.003+2.5057)/5.0*4095.0;	//デジタル値に変換
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
	Now_Time();															//現在時刻取得
	fprintf(fp,"Constant Force Control (2motor) Test Menu\n");					//タイトル出力
	fprintf(fp,"%s\n",filename);										//ファイル名出力
	fprintf(fp,"Sampling Cycle : %8d[msec]\n",Samp_Cycle);				//サンプリング間隔出力
	fprintf(fp,"Record Time : %8.3f[sec]\n\n",Rec_Time);				//サンプリング時間出力
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
//定荷重免荷と左右方向のコンプライアンスを同時制御
void BWS_and_Compliance(void)
{
	int Flag=1;
	int yes;
	int ch;
	
	while(Flag){
		Display_Title();                                            //タイトル表示
		printf(" <BWS_and_Compliance_test>\n");                     //サブタイトル表示
		Sensors_Display();
		
		printf("\n Input Target Force (BWS) (0 - 630)[N]) >> ");			//免荷力入力
		M[2].Target_Force = Double_Stdin(0,630);
		
		printf("\n Input Gain (horizontal) (0 - 1000)[rpm/N]) >> ");
		M[1].Gain1 = Double_Stdin(0,1000);
		
		printf("\n Input Gain (UP) (0 - 1000)[rpm/N]) >> ");            //ゲイン入力（のぼり(ch1)）
		M[2].Gain1 = Double_Stdin(0,1000);
		printf("\n Input Gain (DOWN) (0 - 1000)[rpm/N]) >> ");			//ゲイン入力（くだり(ch2)）
		M[2].Gain2 = Double_Stdin(0,1000);

		printf("\n Input Target Range (horizontal) (0 - 100)[mm]) >> ");         //片側可動域入力
		M[1].Target_Range = Double_Stdin(0,100);
		printf("\n Input Target Range (BWS) (0 - 100)[mm]) >> ");                //片側可動域入力
		M[2].Target_Range = Double_Stdin(0,100);
		
		printf("\n Input Compliance (horizontal) (0.0 - 20.0)[mm/N]) >> ");      //コンプライアンス定数入力
		M[1].Compliance = Double_Stdin(0,20);
		
		Input_Timer_Parameter();                                    //サンプリング間隔と記録時間を入力
		
		M[1].Counter_Past = Count_Read(1);                          //カウンタ初期値
		M[2].Counter_Past = Count_Read(2);
		
		File_Open();												//ファイルオープン
		BWS_and_Compliance_h();
		////////////////////////////////////////////////////////////////////////
		Timer1(Samp_Cycle,Rec_Time,BWS_and_Compliance_drive);			//タイマon
		////////////////////////////////////////////////////////////////////////
		DA_Write(1,2048);											//2.5V出力
		DA_Write(2,2048);
		File_Close();												//ファイルクローズ
		
		sleep(1);
		Savexy();
		Sensors_Display();
		Returnxy();													//カーソル位置復帰
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
	//	int lead=5(BWS),10(Compliance);                            //ボールねじのリード
	//	int slit=2000;                                             //エンコーダの分解能
	//  Reduction_Ratio=12.3;

	Counters_Read();                                               //カウンタ取得
	AD1 = AD_Read(1);
	AD2 = AD_Read(2);                                              //ロードセルよりADにてデジタル値取得
	M[1].Force = AD1/2047.0*1000.0;                                //力［N］の現在値に換算
	M[2].Force = AD2/2047.0*1000.0;
	
	/*ここからコンプライアンス*/
	M[1].Target_Position = -M[1].Compliance * M[1].Force;          //目標位置［mm］算出
	M[1].Position = M[1].Counter*10/2000/Reduction_Ratio;          //現在位置［mm］取得
	
	Dev_Position = M[1].Target_Position-M[1].Position;             //位置偏差［mm］
	
	MaxCounter1 = M[1].Target_Range*2000*Reduction_Ratio/10;       //パルス上限
	MinCounter1 = -M[1].Target_Range*2000*Reduction_Ratio/10;      //パルス下限　
	
	M[1].Input_RPM = M[1].Gain * Dev_Position;                     //位置フィードバックゲインから目標速度算出		
	if((-0.1<=Dev_Position)&&(Dev_Position<=0.1)) M[1].Input_RPM = 0;
	if(M[1].Counter < MinCounter1) M[1].Input_RPM = 0;
	if(MaxCounter1 < M[1].Counter) M[1].Input_RPM = 0;
	
	/*ここから定荷重免荷*/
	
	Dev_Force = M[2].Target_Force - M[ch].Force;                   //力の偏差を計算

	MaxCounter2 = M[2].Target_Range*2000*Reduction_Ratio/5;        //パルスの範囲
	MinCounter2 = -M[2].Target_Range*2000*Reduction_Ratio/5;
	
	if(Dev_Force > 0)	M[2].Input_RPM = M[1].Gain1 * Dev_Force;   //のぼり下りでゲインがちがう
	else				M[2].Input_RPM = M[2].Gain2 * Dev_Force;			
	if(M[2].Counter < MinCounter2) M[2].Input_RPM = 0;
	if(MaxCounter2 < M[2].Counter) M[2].Input_RPM = 0;
	if((-50<=AD2)&&(AD2<=50)) M[ch].Input_RPM = 0; 
    
	/*同時制御*/
	for(ch=1; ch<=Channel; ch++){
		M[ch].DA_Value1 = (M[ch].Input_RPM*0.003+2.5057)/5.0*4095.0;   //デジタル値に変換
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
	Now_Time();                                                         //現在時刻取得
	fprintf(fp,"BWS_and_Compliance\n");                                 //タイトル出力
	fprintf(fp,"%s\n",filename);                                        //ファイル名出力
	fprintf(fp,"Sampling Cycle : %8d[msec]\n",Samp_Cycle);              //サンプリング間隔出力
	fprintf(fp,"Record Time : %8.3f[sec]\n\n",Rec_Time);                //サンプリング時間出力
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
//// １．ステップ応答（モータ） ////////////////////////////////////////////////
void Step_M(void)
{
	int Flag=1;
	int yes;
	int ch;

	while(Flag){
		Display_Title();										//タイトル表示
		printf(" <Step Response Test (Motor) Menu>\n");			//サブタイトル表示
		Counters_Open();
		Counters_Reset();
		Sensors_Display();
		
		printf(" Input Channel (1-2) >> ");							//①チャンネル入力
		ch = Int_Stdin(1,2);
		
		printf("\n Input Target_Pos (0 - 25)[mm]) >> ");			//②目標距離入力
		M[ch].Target_Pos = Double_Stdin(-25,25);
		M[ch].Drive_Pulse = (1000 * M[ch].Target_Pos - M[ch].Abs_External);	//絶対値入力に変更

		printf("\n Do you want to record the Encorder Log ? -- Y or N >> ");
		yes = Yes_or_No();											//ログ取得の有無
		
		if(yes=='y'){
			File_Open();											//ファイルオープン
			Input_Timer_Parameter(ch);								//サンプリング間隔と記録時間を入力
			Step_M_h(ch);											//ログのヘッダ出力
			Savexy();												//カーソル位置取得
			Beep();
			Drive(M[ch].Drive_Pulse,ch);							//駆動
			/////Timer//////////////////////////////////////////////
			Timer(M[ch].Samp_Cycle,M[ch].Rec_Time,Step_M_Log,ch);	//タイマon
			Drive_End();											//駆動終了確認
			
			File_Close();											//ファイルクローズ
		}
		else{
			Savexy();												//カーソル位置取得
			Beep();													//ビープ
			Drive(M[ch].Drive_Pulse,ch);							//駆動
			Drive_End();											//駆動終了確認
		}
		sleep(1);
		Counters_Close();
		Sensors_Display();
		Returnxy();												//カーソル位置復帰
		Godown(2);
		printf("\n Now Drive is End. Again? -- Y or N >> ");
		yes = Yes_or_No();
		if(yes=='n')Flag=0;
	}
}
//// ヘッダ ////
void Step_M_h(int ch)
{
	Now_Time();															//現在時刻取得
	fprintf(fp,"Step Response Test (Motor)\n");							//タイトル出力
	fprintf(fp,"Motor : %d\n",ch);										//ch出力
	fprintf(fp,"%s\n",filename);										//ファイル名出力
	fprintf(fp,"Target_Position : %8.3f[mm]\n",M[ch].Target_Pos);		//目標距離出力
	fprintf(fp,"Basic_Postion : %8.3f[mm]\n",M[ch].Abs_External/1000.0);
	fprintf(fp,"Sampling Cycle : %d[msec]\n",M[ch].Samp_Cycle);			//サンプリング間隔出力
	fprintf(fp,"Record Time : %8.3f[msec]\n\n",M[ch].Rec_Time);			//サンプリング時間出力
	fprintf(fp,"time[msec],Internal,Abs\n");
}
//// ログ ////
void Step_M_Log(int ch)
{
	Counters_Read();													//カウンタ値書き換え
	fprintf(fp,"%8d,%8d,%8d\n",
			M[ch].Samp_Cycle*count, 									//経過時間
			M[ch].Internal+M[ch].Abs_External, 							//内部カウンタ
			M[ch].External+M[ch].Abs_External);							//外部カウンタ絶対値
}
*/
////////////////////////////////////////////////////////////////////////////////
///// その他細かい関数 /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///// エスケープシーケンス /////////////////////////////////////////////////////
void Beep(void)													//ビープ音
{
	printf("\a");
}
void Cls(void)													//画面消去
{
    printf("\x1b[2J");
}
void Clear_Line(void)
{
	printf("\x1b[K");
}
void Gotoxy(int x, int y)										//カーソル絶対位置移動
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
void Goup(int x)												//カーソル相対位置移動（上）
{
	printf("\x1b[%dA", x);
}
void Godown(int x)												//カーソル相対位置移動（下）
{
	printf("\x1b[%dB", x);
}
void Setcolor(int fg, int bg)									//表示色設定
{
    printf("\x1b[1;%d;%dm", fg + 30, bg + 40);
}
void Restore(void)												//初期値に復元
{
    printf("\x1b[0;37;40m");
}
///// 現在時刻取得関数 /////////////////////////////////////////////////////////
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
///// 自作文字入力関数 /////////////////////////////////////////////////////////
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
int Int_Stdin(int a, int b)										//stdinからのint型読み込み　040928
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
float Float_Stdin(float a, float b)									//stdinからのfloat型読み込み　040928
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
double Double_Stdin(double a, double b)								//stdinからのDouble型読み込み　040929
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

///// 低水準ファイル処理関数 ///////////////////////////////////////////////////
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
///// 高水準ファイル処理関数 ///////////////////////////////////////////////////
void File_Open(void)
{
	printf("\n File Name (.csv) >> ");
	gets(filename);												//ファイル名入力
	if((fp=fopen(filename,"a"))==NULL){							//書き込みモード
		printf(" File Open Failed !\n");
		exit(1);
		}
}
void File_Close(void)
{
	fclose(fp);
}
///// タイマパラメータ入力関数 /////////////////////////////////////////////////
void Input_Timer_Parameter(void)
{
	printf("\n Input Sampling Cycle (1-100)[msec] >>");
	Samp_Cycle = Int_Stdin(1,100);
	printf("\n Input Record Time (1-100)[sec] >> ");
	Rec_Time = Double_Stdin(1,100);
}
///// 有限タイマ ///////////////////////////////////////////////////////////////
void Timer1(int Samp_Cycle, double Rec_Time, void(*function)(void))
{
	struct sigevent event;
	struct itimerspec itime;
	timer_t timer_id;
	int chid;
	int rcvid;
	my_message_t msg;
	uint64_t CPS, cycle1, cycle2, ncycles;						//時間計測関連
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
	uint64_t CPS, cycle1, cycle2, ncycles;						//時間計測関連
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
///// 無限タイマ ///////////////////////////////////////////////////////////////
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
				Check_AD_Log();				//あとで修正！！！！！！
			}
		}
	}
}
*/