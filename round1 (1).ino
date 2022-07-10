#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal.h>
#define RST_PIN         47          // Configurable, see typical pin layout above
#define SS_PIN          46          // Configurable, see typical pin layout above
const int rs = 40, en = 41, d4 = 42, d5=43  , d6=44 , d7=45 ;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
MFRC522 mfrc522(SS_PIN, RST_PIN);
const int i=3,j=3;
int OutL=467;
int OutR=391;
int OutLN=816;
int OutRN=793;
int OutMid=801;
int Logic=0;
float Kp = 0.08;
int kd;
int RightPwr=0;
int L=A4;
int LN=A3;
int Mid=A2;
int RN=A1;
int R=A0;
int LeftPwr=0;
int m=0;
int xc=1,yc=1;
int noscan=1;
int ar[i][j]={0,0,0,
              1,1,0,
              0,1,0};
int d=1,x=0,y=0,n=1,e=1,s=1,w=1,opt=0,z=1;
int RtR=3;
int RtF=4;
int LtF=5;
int LtR=6;
int co=0;
int qwerty=0;
int rfco=0;
byte init1[16];
struct ambush
{
    int namb;
    int amb=0;
};
const int grid=3;
const int ht=1;
ambush chk[grid][grid][ht],temp[grid][grid][ht];
void setup()
{
  pinMode(L, INPUT);
  pinMode(LN, INPUT);
  pinMode(Mid, INPUT);
  pinMode(RN, INPUT);
  pinMode(A4, INPUT);
  pinMode(R, INPUT);
  pinMode(RtR, OUTPUT);
  pinMode(RtF, OUTPUT);
  pinMode(LtF, OUTPUT);
  pinMode(LtR, OUTPUT);
  Serial.begin(9600);
  lcd.begin(16, 2);
  SPI.begin();
  mfrc522.PCD_Init();
}

void loop()
{ 
 while (y<i){
          if((m%2)==0){
          Logic=0;
          qwerty=0;
          PPID(0,OutL,OutR,650,640);
          Serial.print("Node");
          Serial.print(x,y);
          analogWrite(RtF,250);                               //Run-straight
          analogWrite(LtF,250);
          delay(20);
          do
          {analogWrite(RtF,30);
          analogWrite(LtF,30);
          qwerty=0;
          scan();
           }while(qwerty==0);    
          do
          {
          analogWrite(RtF,50);
          analogWrite(LtF,50);
          }while(analogRead(L)<OutL||analogRead(R)<OutR);
       analogWrite(LtF,50);
       analogWrite(RtF,50);
       delay(205);
       TurnL();
       analogWrite(LtR,0);analogWrite(RtR,0);
        for (x=0;x<j-1;x++)
        { Logic=0;
          qwerty=0;
          PPID(0,OutL,OutR,650,640);
          Serial.print("Node");
          Serial.print(x,y);
          analogWrite(RtF,250);                               //Run-straight
          analogWrite(LtF,250);
          delay(20);
          do
          {
          analogWrite(RtF,30);
          analogWrite(LtF,30);
          qwerty=0;
          scan();
          }while(qwerty==0);
          do
          {
          analogWrite(RtF,50);
          analogWrite(LtF,50);
          }while(analogRead(L)<OutL||analogRead(R)<OutR);
        }
       analogWrite(LtF,50);
       analogWrite(RtF,50);
       delay(205);
        Turn();
        analogWrite(LtR,0);analogWrite(RtR,0);
        y++;
        m++;
        }
        else
        {
          Logic=0;
          qwerty=0;
         PPID(0,OutL,OutR,650,640);
          Serial.print("Node");
          Serial.print(x,y);
          analogWrite(RtF,250);                               //Run-straight
          analogWrite(LtF,250);
          delay(20);
          do
          {
          analogWrite(RtF,30);
          analogWrite(LtF,30);
          qwerty=0;
          scan();
          }while(qwerty==0);
          do
          {
          analogWrite(RtF,50);
          analogWrite(LtF,50);
          }while(analogRead(L)<OutL||analogRead(R)<OutR);
       analogWrite(LtF,50);
       analogWrite(RtF,50);
       delay(205);
       Turn();
       analogWrite(LtR,0);analogWrite(RtR,0);
        for (x=j-1;x>0;x--)
        { Logic=0;
          qwerty=1;
          PPID(0,OutL,OutR,650,640);
          Serial.print("Node");
          Serial.print(x,y);
          analogWrite(RtF,250);                               //Run-straight
          analogWrite(LtF,250);
          delay(20);
          do
          {
          analogWrite(RtF,30);
          analogWrite(LtF,30);
          qwerty=0;
          scan();
          }while(qwerty==0);
          do
          {
          analogWrite(RtF,50);
          analogWrite(LtF,50);
          }while(analogRead(L)<OutL||analogRead(R)<OutR);
        }
       analogWrite(LtF,50);
       analogWrite(RtF,50);
       delay(205);
        TurnL();
        analogWrite(LtR,0);analogWrite(RtR,0);
        y++;
        m++;
        }
}
analogWrite(RtF,0);
analogWrite(LtR,0);
analogWrite(RtR,0);
analogWrite(LtF,0);
//Ambush-Detector

 z=3;x=i-1;y=j-1;n=1;e=1;s=1;w=1;
 while((x>0)||(y>0))
 {
  Serial.print(x);
  Serial.println(y);
  opt=0;n=1;e=1;s=1;w=1;
  d=z;
  if(x>0)
  if(ar[x-1][y]==0)
  {n=0;opt++;}
  if(y>0)
  if(ar[x][y-1]==0)
  {w=0;opt++;}
  if(5>x)
  if(ar[x+1][y]==0)
  {s=0;opt++;}
  if(5>y)
  if(ar[x][y+1]==0)
  {e=0;opt++;}
  if(opt==1)
  {if((x!=5)&&(y!=5)){ar[x][y]=2;d=4;}}
  if(w==0&&(d!=1))
  {y--;z=3;continue;}
  if(n==0&&(d!=2))
  {x--;z=0;continue;}
  if(e==0&&(d!=3))
  {y++;z=1;continue;}
  if(s==0&&(d!=0))
  {x++;z=2;continue;}
  } 
 x=i-1;y=j-1;
  d=3;
 while(x>0||y>0)
 {
  if(5>x)
  if((ar[x+1][y]==0)&&((d%4)!=0))
  {z=2;}
  if(5>y)
  if((ar[x][y+1]==0)&&((d%4)!=3))
  {z=1;}
  if(x>0)
  if((ar[x-1][y]==0)&&((d%4)!=2))
  {z=0;}
  if(y>0)
  if((ar[x][y-1]==0)&&((d%4)!=1))
  {z=3;}
  if((((d%4)==1)&&(z==0))||(((d%4)==2)&&(z==1))||(((d%4)==3)&&(z==2))||(((d%4)==0)&&(z==3)))
  {analogWrite(LtF,50);
       analogWrite(RtF,50);
       delay(205);TurnL();analogWrite(LtR,0);analogWrite(RtR,0);}
  else
  while((d%4)!=z)
  {analogWrite(LtF,50);
       analogWrite(RtF,50);
       delay(205);Turn();analogWrite(LtR,0);analogWrite(RtR,0);}
   Logic=0;
   PPID(0,OutL,OutR,650,640);
          Serial.print(x,y);
          analogWrite(RtF,250);                               //Run-straight
          analogWrite(LtF,250);
          delay(20);
          do
          {
          analogWrite(RtF,50);
          analogWrite(LtF,50);
          }while(analogRead(L)<OutL||analogRead(R)<OutR);                                //Stop
          if(z==1){y++;}
          if(z==2){x++;}
          if(z==3){y--;}
          if(z==0){x--;}
 }
delay(100000000);
}
void PPID(int Threshold,int OutL, int OutR, int T1, int T2)
    {
    while(Logic==0)
    {
    Direction();
    if(T1>analogRead(Mid)>T2)
    {Kp=0.13;
    kd=130;}
    else
    {
    Kp=0.15;
    kd=100;}
    Kp=map(analogRead(Mid),500,1000,50,200);
    Kp=(float)(Kp/1000);
    kd=map(analogRead(Mid),500,1000,90,70);
    RightPwr = ((((analogRead(RN)-analogRead(LN))-Threshold) * Kp)+kd);
    LeftPwr = (((Threshold - (analogRead(RN)-analogRead(LN))) * Kp)+kd);
    if(RightPwr<0)
    {
      RightPwr=0;}
      if(LeftPwr<0)
      {
       
        LeftPwr=0;
        }
    analogWrite(RtR,0);
    analogWrite(RtF,RightPwr);
    analogWrite(LtR,0);
    analogWrite(LtF,LeftPwr);
    if((analogRead(L)<(OutL))||(analogRead(R)<(OutR)))
    Logic=1;
    }
    }
void Turn()
{      
       analogWrite(LtF,50);
       analogWrite(RtF,50);
       analogWrite(LtF,0);
       analogWrite(RtR,0);
       analogWrite(LtR,60);
       analogWrite(RtF,60);
       delay(400);
       while((analogRead(RN)>(OutRN))&&(analogRead(Mid)>(OutMid)))
       {
       analogWrite(LtR,50);
       analogWrite(RtF,50);
       }
       delay(50);
       //analogWrite(LtR,70);
       //analogWrite(RtF,70);
       analogWrite(LtR,0);
       analogWrite(RtF,0);
       d++;
}
void TurnL()
{
       analogWrite(LtF,50);
       analogWrite(RtF,50);
       analogWrite(RtF,0);
       analogWrite(LtR,0);
       analogWrite(RtR,60);
       analogWrite(LtF,60);
       delay(400);
       while((analogRead(LN)>(OutLN))&&(analogRead(Mid)>(OutMid)))
       {
       analogWrite(RtR,50);
       analogWrite(LtF,50);
       }
       delay(50);
       analogWrite(RtR,0);
       analogWrite(LtF,0);
       d--;
}
void Direction()
{
 switch((d%4))
 {
 case 1:Serial.println("E");break;
 case 2:Serial.println("S");break;
 case 3:Serial.println("W");break;
 case 0:Serial.println("N");break;
 }
}
void scan()
{
 MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  byte block;
  byte len;
  MFRC522::StatusCode status;

  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  Serial.println(F("**Card Detected:**"));

  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid));


  Serial.print(F("Name: "));

  byte buffer1[18];

  block = 4;
  len = 18;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); 
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Authentication failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  co=0;
  for (uint8_t i = 1; i < 16; i++)
  {
    if (buffer1[i] != 32)
    {
      init1[co++]=buffer1[i];
    }
  }
  
  Serial.print(" ");
  lcd.clear();
  String nmb=(char*)init1;
  byte buffer2[18];
  block = 1;

  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(mfrc522.uid)); 
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Authentication failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  status = mfrc522.MIFARE_Read(block, buffer2, &len);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
byte init2[16];

  //PRINT LAST NAME
  for (uint8_t i = 0; i < 16; i++) {
    init2[i]=buffer2[i];
  }
  lcd.clear();
  String nmb2=(char*)init2;
    //lcd.print(nmb);
      lcd.setCursor(0,1);
    lcd.print(nmb2);
    int nambus=nmb2[0]-48;  //actual data
  if(noscan)
{
  Serial.println(F("\n**End Reading**\n"));
  qwerty=1;
  analogWrite(LtR,0);
  analogWrite(RtR,0);
  analogWrite(RtF,0);
  analogWrite(LtF,0);
  if(rfco<6)
  {
   delay(2000);
   rfco++; 
   }
  /* lcd.clear();
   lcd.print(grid-xc);
   lcd.print(grid-yc);
   */
   if(yc%2==1)
   {{
    chk[grid-xc][grid-yc][0].namb=nambus;
   
    xc++;
   if(xc==grid+1)
   {xc=grid;
    yc++;
   }
   }}
   else
   {
    chk[grid-xc][grid-yc][0].namb=nambus;
    xc--;
   if(xc==0)
   {
    xc=1;
    yc++;
   }
   }}
   if(yc==grid+1)
   {yc++;
   delay(3000);
   ambushfind();
   noscan=0;
   }
  //delay(); //change value if you want to read cards faster

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1(); 
}
void ambushfind()
{
    int s=grid,h=ht,found1=1,found2=1,cert=2;
    /*cout<<"Enter the the no. of node in one lane :";
    cin>>s;
    cout<<"Enter the no. of height points :";
    cin>>h;*/
   /* for(int k=0;k<h;k++)
    {
        for(int i=0;i<s;i++)
        {
            for(int j=0;j<s;j++)
            {
                cout<<"Enter near ambush for ("<<j<<","<<i<<","<<k<<"):";
                cin>>chk[j][i][k].namb;
            }
        }
    }*/
    chk[0][0][0].amb=2;
    chk[grid-1][grid-1][0].amb=2;
    //chk[5][4].amb=2;
    again:
    found1=1;
    found2=1;
    while(found1)
    {
        found1=0;
        while(found2)
        {found2=0;
        for(int z=0;z<h;z++)
        {
            for(int i=0;i<s;i++)
            {
                for(int j=0;j<s;j++)
                {
                    if(!chk[j][i][z].amb)
                    {
                        int ambn[6][3]={{j+1,i,z},{j-1,i,z},{j,i+1,z},{j,i-1,z},{j,i,z+1},{j,i,z-1}};
                        for(int k=0;k<6;k++)
                        {
                            if(ambn[k][0]>=s||ambn[k][0]<0||ambn[k][1]>=s||ambn[k][1]<0||ambn[k][2]>=h||ambn[k][2]<0)
                                continue;
                            int sum=0,ambadd[6][3]={{ambn[k][0]+1,ambn[k][1],ambn[k][2]},{ambn[k][0]-1,ambn[k][1],ambn[k][2]},{ambn[k][0],ambn[k][1]+1,ambn[k][2]},{ambn[k][0],ambn[k][1]-1,ambn[k][2]},{ambn[k][0],ambn[k][1],ambn[k][2]+1},{ambn[k][0],ambn[k][1],ambn[k][2]-1}};
                            for(int l=0;l<6;l++)
                            {
                                if(ambadd[l][0]>=s||ambadd[l][0]<0||ambadd[l][1]>=s||ambadd[l][1]<0||ambadd[l][2]>=h||ambadd[l][2]<0)
                                    continue;
                                if(!chk[ambadd[l][0]][ambadd[l][1]][ambadd[l][2]].amb)
                                    sum++;
                            }
                            if(sum==chk[ambn[k][0]][ambn[k][1]][ambn[k][2]].namb)
                            {
                                for(int m=0;m<6&&chk[j][i][z].amb==0;m++)
                                {
                                    if(ambn[m][0]>=s||ambn[m][0]<0||ambn[m][1]>=s||ambn[m][1]<0||ambn[m][2]>=h||ambn[m][2]<0)
                                    continue;
                                    chk[ambn[m][0]][ambn[m][1]][ambn[m][2]].namb--;
                                }
                            found2=1;
                            chk[j][i][z].amb=1;
                            }
                        }
                    }
                }
            }
            }
        }
        for(int z=0;z<h;z++)
        {
            for(int i=0;i<s;i++)
            {
                for(int j=0;j<s;j++)
                {
                    if(!chk[j][i][z].amb)
                    {
                        int ambn[6][3]={{j+1,i,z},{j-1,i,z},{j,i+1,z},{j,i-1,z},{j,i,z-1},{j,i,z+1}};
                        for(int k=0;k<6;k++)
                        {
                            if(ambn[k][0]>=s||ambn[k][0]<0||ambn[k][1]>=s||ambn[k][1]<0||ambn[k][2]>=h||ambn[k][2]<0)
                                continue;
                            if(!chk[ambn[k][0]][ambn[k][1]][ambn[k][2]].namb)
                            {
                                chk[j][i][z].amb=2;
                                found1=1;
                                found2=1;
                            }
                        }
                    }
                }
            }
        }

    }

    /*for(int i=0;i<s;i++)
    {
        for(int j=0;j<s;j++)
        {
            if(chk[j][i].namb)
            {
                if(cert==2)
                {
                    for(int k=0;k<s;k++)
                    {
                        for(int l=0;l<s;l++)
                        {
                            temp[l][k].amb=chk[l][k].amb;
                            temp[l][k].namb=chk[l][k].namb;
                        }
                    }
                    chk[1][0].amb=2;
                    cert=1;
                    goto again;
                }
                else if(cert==1)
                {
                    for(int k=0;k<s;k++)
                    {
                        for(int l=0;l<s;l++)
                        {
                            chk[l][k].amb=temp[l][k].amb;
                            chk[l][k].namb=temp[l][k].namb;
                        }
                    }
                    chk[0][1].amb=2;
                    cert=0;
                    goto again;
                }
            }
        }
    }*/
    //cout<<endl;
    for(int z=0;z<h;z++)
    {
        for(int i=0;i<s;i++)
        {
            for(int j=0;j<s;j++)
            {
                if(chk[j][i][z].amb==2)
               chk[j][i][z].amb=0;
               if(chk[j][i][z].amb==1)
               {
                lcd.clear();
                lcd.print("(");
                lcd.print(grid-j);
                lcd.print(",");
                lcd.print(grid-i);
                lcd.print(")");
                delay(2000);
               }
            }
        }
    }
    //cout<<endl;
    for(int z=0;z<h;z++)
    {
        for(int i=0;i<s;i++)
        {
            for(int j=0;j<s;j++)
            {
                ar[j][i]=chk[j][i][z].amb;
                //cout<<"("<<j<<","<<i<<","<<z<<")-"<<chk[j][i][z].namb<<"  |  "<<chk[j][i][z].amb<<"\n";
            }
        }
    }
}
