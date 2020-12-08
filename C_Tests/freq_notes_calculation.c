#include <stdio.h>
#include <math.h>

#define FREQ 36000000
#define NS 500
#define NUMBER_OF_NOTES 60

// float freq_notes [] = {65,4169,299481002041573,420242579916177,78603739232882,411435873623787,31187486126292,503709114824298,0042658985038103,83190280924110,006069043491116,547370307119123,477637585019130,82138,598962004083146,840485159832155,572074784656164,822871747247174,623749722524185,007418229648196,008531797008207,66380561848220,012138086982233,094740614239246,955275170039261,63277,19293,67311,13329,63349,23370392415,31440,01466,17493,89523,26554,37587,34622,26659,27698,47740784830,62880,01932,34987,781046,521108,751174,681244,531318,531396,9414801568,011661,251760,031864,691975,57};
float freq_note_estimate = 0.00;

int main(){
    int i = 0, minimum_psc = 0, minimum_arr = 0;
    float freq_notes [NUMBER_OF_NOTES] = {0}, relative_error = 0.00, minimum_error = 1;

    for(i = 0; i < NUMBER_OF_NOTES; i++){
        freq_notes[i] = 65.41*pow(2,i*0.08333);
    }
    for(int note = 0; note < NUMBER_OF_NOTES; note++){
        minimum_error = 0.1;
        for(int psc = 1; psc < 1000; psc++){
            for(int arr = 1; arr < 1000; arr++){
                // printf("PSC: %d\n", psc);
                freq_note_estimate = (float)(FREQ/((float)psc*NS*(float)arr));
                relative_error = ((float)(fabs(freq_notes[note] - freq_note_estimate))/freq_note_estimate);
                if(relative_error < minimum_error){
                    minimum_error = relative_error;
                    minimum_psc = psc;
                    minimum_arr = arr;
                }
                // printf("%lf\n", freq_note_estimate);
            }
        }
        printf("Minimum for %.2lf: PSC: %d, ARR: %d and Error of %.4lf\n", freq_notes[note], minimum_psc, minimum_arr, minimum_error);
    }

}

