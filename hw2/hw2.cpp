#include <iostream>
using namespace std;

enum weather{sunny=0, cloudy, rainy};

weather nextweather(weather);
void distribution(void);

int main(){
    srand(time(NULL));
    distribution();
    return 0;
}

weather nextweather(weather today){
    double transition[3][3]={
        {0.8, 0.2, 0},
        {0.4, 0.4, 0.2},
        {0.2, 0.6, 0.2}
    };
    double num = (double)(rand()%100) / 100;
    if(num < transition[today][0]) return sunny;
    else if(num < transition[today][0]+transition[today][1]) return cloudy;
    else return rainy;
    
}

void distribution(void){
    //Run N times to find the weather of a random day
    int sunny_count=0, cloudy_count=0, rainy_count=0;
    int N = 10000, length=10000;
    for(int i=0; i< N; i++){
        weather w = (weather)(rand()%3);
        for(int j =0; j<length; j++){
            if((weather)w == sunny) sunny_count++;
            else if((weather)w == cloudy) cloudy_count++;
            else rainy_count++;
            w = nextweather(w);
        }
    }
    cout << "stationary distribution" << endl;
    cout << "Prob. " << "sunny: "<< ((double)sunny_count / (N* length)) << endl;
    cout << "Prob. " << "cloudy: "<< ((double)cloudy_count / (N* length)) << endl;
    cout << "Prob. " << "rainy: "<< ((double)rainy_count / (N* length)) << endl;
}
