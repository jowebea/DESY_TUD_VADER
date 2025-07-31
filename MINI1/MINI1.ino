#include <Controllino.h>
#include <RunningMedian.h>

/*
###################################################################################################

 MINI1 reads the pressure in the manifold and sends it ever ~10ms via Serial USB in csv style text. 

 to synchronize with the Detector it waits for "t0" - the time of an rising TTL Signal (24V)


 TESTED on 9.11.2025 at DESY -> works

 ToDo: use the interupt Pin (IN0 or IN1) of Controllino to recieve TTL 
 Goal: To minimize the delay between TTL and the time of t0, one should use an interupt on rising TTL pulse.
       current delay < 2ms, probably <0.5ms
 The usage of A0 as TTL reciever was a fix, since IN0 and IN1 did not worked.
 (The LED of IN0/IN1 indicated the TTL, but analogRead returned 0 and Interuped was not called
    I tried: TTL = [CONTROLLINO_IN0, CONTROLLINO_IN0, 5,6,7]) 
    The 5...7 I tried since a forum indicated that there migth be a mislabeling in the lib

###################################################################################################
*/

RunningMedian  _p21 = RunningMedian(11);
RunningMedian  _p20 = RunningMedian(11);



#define PT100 CONTROLLINO_A2
#define P20 CONTROLLINO_A1
#define P21 CONTROLLINO_A0
#define TTL CONTROLLINO_IN0 //only Arduino PIN 2,3 are capabel of interups

float mapP20(float p20Value) {
    // Lineares Modell: y = a + b*x
    // a = 2.63, b = 0.659
    return 2.63 + 0.659 * p20Value;
}

float mapP21(float p21Value) {
    // Lineares Modell: y = a + b*x
    // a = 9.95, b = 3.59
    return 9.95 + 3.59 * p21Value;
}

int read_fast_p21(){
    // Signal (S) -> P in kPa
    // P = 9.7016 + (3.2327 * S)  
    // S(6) = P(29)kPa ; S(401) = P(1306)kPa      
    //return map(_p21.getMedian(),6,401, 29, 1306);
    return mapP21(analogRead(P21));
}

int read_accurate_p21(){
    // sample median ADC reading
    for (size_t i = 0; i < 12; i++)
    {
        _p21.add(analogRead(P21));
    }
    // Signal (S) -> P in kPa
    // P = 9.7016 + (3.2327 * S)  
    // S(6) = P(29)kPa ; S(401) = P(1306)kPa      
    return mapP21(_p21.getMedian());
}

int read_accurate_p20(){
    // sample median ADC reading
    for (size_t i = 0; i < 12; i++)
    {
        _p20.add(analogRead(P20));
    }        
    // Signal (S) -> P in kPa
    // P = 2,9872 + (0,6029 * S)
    // S(0) = P(3)kPa ; S(400) = P(244)kPa  
    return mapP20(_p20.getMedian());
}

int read_p(){
    int P = read_fast_p21();
    if (P< 150) // kPa
    {
        P = read_accurate_p20();
    }
    else
    {
        P = read_accurate_p21();
    }
    return P;
}

// ## Setup & Loop ##

void setup()
{
    Serial.begin(115200);
    pinMode(PT100, INPUT);
    pinMode(P20, INPUT);
    pinMode(P21, INPUT);
    pinMode(TTL, INPUT);
}

void loop()
{    
    int p = read_p(); // read P21 (1ms) if P<150kPa then read P20 (1ms)    
    Serial.println(p);
}

