
#include<SoftwareSerial.h>
SoftwareSerial gps(11, 12);



byte gps_set_sucess = 0 ;
byte bytegps = 0;                    /////////// GPS
char datagps[100]="";
char hora[15]="";
char lat[15]="";
char lon[15]="";
char NS[5]="";
char WE[5]="";
char sat[5]="";
char alt[20]="";
int comas[15];
int contcomas = 0;
int i=0;
float lat_degInt;
float lat_degDec;
float alt_easy;
float hora_easy1;
float hora_easy2;
float lat_easy;
float long_degInt;
float long_degDec;
float long_easy;
bool gps_state = false;





void setup() {                                  //////////////////////////////////SETUP

 Serial.begin(9600);                          ////////////GPS

 gps.begin(9600);
 pinMode(13,OUTPUT);
 gps_flightmode();

                
 
}

void loop() {                            /////////////////////////////////////////////////LOOP

  memset(datagps,0,sizeof(datagps));          /////////////////////GPS
  gps_state = false;
  
  while(datagps[4] != 'G'){
  gpsloop();
  }
  

  gpsdatatreatment();
  latlondeg();


  // Print in compu to debug
  Serial.print(hora[0],0);
  Serial.print(hora[1],0);
  Serial.print(":");
  Serial.print(hora[2],0);
  Serial.print(hora[3],0);
    Serial.print(":");
  Serial.print(hora[4],0);
  Serial.print(hora[5],0);
  Serial.print(";");
  if(NS[0]=='S'){
   Serial.print("S");
  }
  else Serial.print("N");

  Serial.print(lat_easy,4);
 Serial.print(";");
  if(WE[0]=='W'){
   Serial.print("W");
  }
  else Serial.print("E");
  Serial.print(long_easy,4);
  Serial.print(";");
  Serial.println(alt_easy,0);

  delay(1000);



  

  clean_all();

if(gps_state == false) {pinMode(13,HIGH);}



}

///////////////////////////////////////GPS LOOP
void gpsloop(){


gps.flush();

gps.begin(4800);

memset(datagps,0,sizeof(datagps));

bytegps = 0;
int i = 1;

while(bytegps != '$'){

bytegps = gps.read();

}


datagps[0] = '$';
bytegps = 0;

//   while(bytegps != '*' ){
//   bytegps = gps.read();
//   if(bytegps!=255){
//   datagps[i] = bytegps;
//   i++;}
// }

while(bytegps!='$'){
bytegps = gps.read();
if(bytegps != 255 && bytegps!='$'){
datagps[i]=bytegps;
i++;
}


}


}
///////////////////////////////////////END GPS LOOP


///////////////////////////////////////GPS DATA TREATMENT

void gpsdatatreatment() {

 contcomas = 0;//es la posicio del vector comas
 // "i" sera la posicio de la coma dins del vector datagps

 for (i=7 ; i<sizeof(datagps) ; i++){
 //Serial.println(i);
   if (datagps[i]==','){

   comas[contcomas]=i;
   contcomas++;

   } // end if
 } // end for


 for(i=7;i<comas[0]-3;i++){
 hora[i-7]= datagps[i];
 }

 for(i=comas[0]+1;i<comas[1];i++){
 lat[i-comas[0]-1]= datagps[i];
 }

 for(i=comas[1]-1;i<comas[2];i++){
 NS[i-comas[1]-1]= datagps[i];
 }

 for(i=comas[2]+1;i<comas[3];i++){
 lon[i-comas[2]-1]= datagps[i];
 }

 for(i=comas[3]+1;i<comas[4];i++){
 WE[i-comas[3]-1]= datagps[i];
 }

 for(i=comas[5]+1;i<comas[6];i++){
 sat[i-comas[5]-1]= datagps[i];
 }

 for(i=comas[7]+1;i<comas[8];i++){
 alt[i-comas[7]-1]= datagps[i];
 }

}

///////////////////////////////////////END GPS DATA TREATMENT

///////////////////////////////////////CLEAN ALL

void clean_all() {

memset(hora,0,sizeof(hora));
memset(lat,0,sizeof(lat));
memset(lon,0,sizeof(lon));
memset(NS,0,sizeof(NS));
memset(WE,0,sizeof(WE));
memset(sat,0,sizeof(sat));
memset(alt,0,sizeof(alt));

}

///////////////////////////////////////END CLEAN ALL

///////////////////////////////////////CAMBIO A GRADOS

void latlondeg() {

hora_easy1=float(atof(hora[0]));
hora_easy2=float(atof(hora[1]));
alt_easy=float(atof(alt));
lat_degInt = float(int(atof(lat)/100));
lat_degDec = float((atof(lat)-lat_degInt*100)/60);
lat_easy = lat_degInt + lat_degDec;
long_degInt = float(int(atof(lon)/100));
long_degDec = float((atof(lon)-long_degInt*100)/60);
long_easy = long_degInt + long_degDec;

}




///////////// GPS flight mode and disable all NMEA sentences but GPGGA

void gps_flightmode(){
  gps_set_sucess = 0 ;     
  gps.begin(9600); 
  // START OUR SERIAL DEBUG PORT
  Serial.begin(9600);
  Serial.println("GPS Level Convertor Board Test Script");
  Serial.println("03/06/2012 2E0UPU");
  Serial.println("Initialising....");
  //
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
  // THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
  //
  gps.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
  gps.begin(4800);
  gps.flush();

  //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  Serial.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                        };
    while(!gps_set_sucess)
    {
      sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
      gps_set_sucess=getUBX_ACK(setNav);
    }
    gps_set_sucess=0;

  // THE FOLLOWING COMMANDS DO WHAT THE $PUBX ONES DO BUT WITH CONFIRMATION
  // UNCOMMENT AS NEEDED
  
  Serial.println("Switching off NMEA GLL: ");
  uint8_t setGLL[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B                   };
   while(!gps_set_sucess)
   {    
     sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
     gps_set_sucess=getUBX_ACK(setGLL);
   }
   gps_set_sucess=0;
   Serial.println("Switching off NMEA GSA: ");
    delay (500);
   uint8_t setGSA[] = { 
     0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32                   };
     while(!gps_set_sucess)
     {  
       sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
       gps_set_sucess=getUBX_ACK(setGSA);
     }
     gps_set_sucess=0;
     Serial.println("Switching off NMEA GSV: ");
     delay (500);
     uint8_t setGSV[] = { 
       0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39                   };
       while(!gps_set_sucess)
       {
         sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
         gps_set_sucess=getUBX_ACK(setGSV);
       }
       gps_set_sucess=0;
//   Serial.print("Switching off NMEA RMC: ");
//   uint8_t setRMC[] = { 
//   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40                   };
//   while(!gps_set_sucess)
//   {
//   sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
//   gps_set_sucess=getUBX_ACK(setRMC);
//   }

}




// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gps.write(MSG[i]);
    Serial.print(MSG[i], HEX);
     delay (500);
  }
  gps.println();
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
  delay (500);

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
       delay (500);
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
       delay (500);
      return false;
    }

    // Make sure data is available to read
    if (gps.available()) {
      b = gps.read();
       delay (500);

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
        delay (500);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }

    }
  }
}
