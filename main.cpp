#include "yaws.h"


int main(void) 
{
    // Create the YAWS
    Yaws weatherStation;

    // Starts the weatherStation operation
    weatherStation.run();

    while(true){
        printf("YAWS has failed! \n");
    }
}
