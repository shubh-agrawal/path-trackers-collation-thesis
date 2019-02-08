    #include <SPI.h>
    #include <nRF24L01.h>
    #include <RF24.h>
    #include <ros.h>
    #include <std_msgs/Float32.h>

    ros::NodeHandle nh;

    std_msgs::Float32 left;
    std_msgs::Float32 right;
    //std_msgs::Float32 avg;
    ros::Publisher chatter1("l_vel", &left);
    ros::Publisher chatter2("r_vel", &right);
    //ros::Publisher chatter3("avg", &avg);
        
    RF24 radio(9,10)  ; // CNS, CE
    //const byte address[6] = "10001";
    const byte address = 0xf0f0f0f0c3;
    
      
    void setup() 
    {
      nh.initNode();
      nh.advertise(chatter1);
      nh.advertise(chatter2);
     // nh.advertise(chatter3);
      Serial.begin(57600);
      radio.begin();
      radio.openReadingPipe(1, address);
      radio.setPALevel(RF24_PA_MIN);
      radio.startListening();
     
    }
    
    void loop() 
    {
      if (radio.available()) 
      { 
        //Serial.println("abc");
        float data[2];
        
        radio.read(&data, sizeof(data));
          
        //Serial.println(data[0]);

        if(data[0] == 1)
        {
          left.data = data[1]* (2 * 3.14 * 0.16 / 60);
          chatter1.publish(&left);
          //Serial.print("Left_vel:");
          //Serial.println(data[1]);
        }
        if(data[0] ==2){
//          Serial.println("Received");

          right.data = -data[1]* (2 * 3.14 * 0.16 / 60);
          chatter2.publish(&right);
          //Serial.print("Right_vel:");
          //Serial.println(data[1]);
        }
        
      }
     // avg.data = (left.data + right.data)/2;
      //chatter3.publish(&avg);

      nh.spinOnce();
   
    }
