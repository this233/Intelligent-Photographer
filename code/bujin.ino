#define ACM_0 Serial  //连接ROS
#define directionPin 2
#define stepPin 3

const int pulsePerRev = 200;  //每转脉冲数
const float mmPerRev = 6.0;     //每转移动距离
const String noMove = "N000/n";
const int pulseDelay=300;//控制速度

void move(const float length,const bool direction) {
  float revCount = length / mmPerRev;       //圈数
  int pluseCount = revCount * pulsePerRev;  //脉冲数
  for (int i = 0; i < pluseCount; i++) {
    if (direction) {  //向上
      digitalWrite(directionPin, HIGH);
    } else {  //向下
      digitalWrite(directionPin, LOW);
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay);//控制速度
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay);//控制速度
  }
}  
//长度单位 mm

String ReadMsg(HardwareSerial& serial) {
  delay(5);
  //ACM_0.print("recieved");
  while (1) {
    if (ACM_0.available() > 0) {
      //ACM_0.print("recieved msg");
      break;
    }
  }
  while (serial.available() > 0) {
    if ((char)serial.read() == '$') {
      String con;
      int length = 5;
      while (length > 0) {
        if (serial.available() > 0) {
          char c = serial.read();  // 读取一个字符
          con += c;
          length--;
          if (c == '\n') {
            // ACM_0.print("$");
            // ACM_0.print("recieved:");
            // ACM_0.println(con);
            return con;
          }
        }
      }
      return noMove;
    }
  }
  // ACM_0.print("$");
  // ACM_0.print("recieved nothing");
  return noMove;
} 
//获取串口控制信号
//消息格式$U0*0\n
//第一位$；第二位U/D，3-5位移动距离（mm）

void setup() {
  //ACM_0.println("start0");
  ACM_0.begin(9600);  //USB波特率9600
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  delay(500);
}

void loop() {
  //ACM_0.println("start");
  String mode = ReadMsg(ACM_0);
  //ACM_0.println(mode);
  bool direction = 1;
  float length = 0.0;
  char c = mode[0];
  //ACM_0.println(c);
  if (c == 'u' || c == 'U') {  //向上
    direction = true;
  } else if (c == 'd' || c == 'D') {  //向下
    direction = false;
  }
  //ACM_0.println(mode.length());
  for(int i=1;i<mode.length()-1;i++){
    int ch=(mode[i]-'0');
    length=length*10+ch;
  }
 //ACM_0.println(length);
  if(c=="N"){
    length=0;
  }
 // ACM_0.println(length);
  //delay(5);
 // ACM_0.println(direction);  
  move(length, direction);
  delay(500);
  ACM_0.print("$");  //完成后回复
  //while(1);
  //return;
}