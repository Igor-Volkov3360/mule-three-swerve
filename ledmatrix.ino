#include <RGBmatrixPanel.h>

#define CLK 8
#define OE 9
#define LAT 10
#define A A0
#define B A1
#define C A2 

#define select1 11
#define select2 12
#define select3 13

RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, false);

// Similar to F(), but for PROGMEM string pointers rather than literals
#define F2(progmem_ptr) (const __FlashStringHelper *)progmem_ptr

int rand(int);
const char teamScroll[] PROGMEM = "3360 HYPERION 3360";
const char ez[] PROGMEM = "2ez";
const char teamNumber[] PROGMEM = "3360";
const char hyperion[] PROGMEM = "HYPERION";

int color3360 = 0;
int scrollAlternX = 0;
int16_t teamX = matrix.width(),
        teamMin = (int16_t)sizeof(teamScroll) * -12,
        hueTeam = 140;

int16_t ezX = matrix.width(),
        ezMin = (int16_t)sizeof(ez) * -12,
        hueez = 140;

int jaune[] = { 255, 140, 0 };
int orange[] = { 255, 90, 0 };
int mauve[] = { 100, 0, 255 };
int vert[] = { 0, 255, 0 };
int rouge[] = { 255, 0, 0 };
int bleu[] = { 0, 0, 255 };

int select1Val = 0;
int select2Val = 0;
int select3Val = 0;
int allVal = 0;

void setup() {
  matrix.begin();
  matrix.setTextWrap(false);  // Allow text to run off right edge
  matrix.setTextSize(2);

  //pinMode(select1, INPUT);
  //pinMode(select2, INPUT);
  //pinMode(select3, INPUT);
  
}

void loop() {
  setScrollAltern();
  
    /*
  select1Val = digitalRead(select1);
  select2Val = digitalRead(select2)<<1;
  select3Val = digitalRead(select3)<<2;

  allVal = select1Val + select2Val + select3Val;
  Serial.println(allVal);
  Serial.println();

  switch(allVal){
    case 0:
      setTeam();
      break;

    case 1:
      set3360();
      break;

    case 2:
      setOrange();
      break;

    case 3:
      setPurple();
      break;

    case 4:
      setRed();
      break;

    case 5:
      setGreen();
      break;

    case 6:
      setBlue();
      break;

  }
*/
  matrix.swapBuffers(true);
  delay(10);
}

void setGreen() {
  for (int x = 0; x < 32; x++)
    for (int y = 0; y < 16; y++)
      matrix.writePixel(x, y, matrix.Color333(vert[0], vert[1], vert[2]));
}

void setRed() {
  for (int x = 0; x < 32; x++)
    for (int y = 0; y < 16; y++)
      matrix.writePixel(x, y, matrix.Color333(rouge[0], rouge[1], rouge[2]));
}

void setPurple() {
  for (int x = 0; x < 32; x++)
    for (int y = 0; y < 16; y++)
      matrix.writePixel(x, y, matrix.Color333(mauve[0], mauve[1], mauve[2]));
}

void setOrange() {
  for (int x = 0; x < 32; x++)
    for (int y = 0; y < 16; y++)
      matrix.writePixel(x, y, matrix.Color333(orange[0], orange[1], orange[2]));
}

void setTeam() {
  matrix.fillScreen(0);

  matrix.setTextColor(matrix.Color333(orange[0], orange[1], orange[2]));
  matrix.setCursor(teamX, 1);
  matrix.print(F2(teamScroll));

  if ((--teamX) < teamMin) teamX = matrix.width();
}

void set2ez() {
  matrix.fillScreen(0);

  matrix.setTextColor(matrix.Color333(255, 0, 0));
  matrix.setCursor(0, 0);
  matrix.print(F2(ez));
}

void setBlue() {
 for (int x = 0; x < 32; x++)
    for (int y = 0; y < 16; y++)
      matrix.writePixel(x, y, matrix.Color333(bleu[0], bleu[1], bleu[2]));
}

void set3360() {
  matrix.fillScreen(0);
  matrix.setTextSize(1);
  matrix.setTextColor(matrix.Color333(orange[0], orange[1], orange[2])/*ColorHSV(color3360, 255, 255, false)*/);
  matrix.setCursor(4, 4);
  matrix.print(F2(teamNumber));

  //color3360++;
  //if(color3360 > 1500) color3360 = 0;
}

void setScrollAltern() {
  matrix.fillScreen(0);
  matrix.setTextSize(1);
  matrix.setTextColor(matrix.ColorHSV(130, 100, 200, false));
  matrix.setCursor(scrollAlternX, 4);
  matrix.print(F2(hyperion));
  if ((--scrollAlternX) < teamMin) scrollAlternX = matrix.width();
}



