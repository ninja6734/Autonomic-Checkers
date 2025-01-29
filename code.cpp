//defines
#define CM 178
#define BIT_PRECISION 2

//action: first bit = 1: new action 0: old action
//action: remaining bits: position
#define POS_BIT 14
#define ACT_BYTE 56

//includes
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

/*
 * Port Includes: 
 * 8 - MotorController, 1
 * 7 - MotorController, 2
 * 12 - MotorController, 3
 * 13 - MotorController, 4
 * 2 - Button, SelRight
 * 3 - Button, SelLeft
 * 4 - Button, Enter
 * 5 - Magnet
 * A5 - LCD, SDA
 * A4 - LCD, SCL
 * 5V - LCD, VCC
 */

//bits needed for one action: 8
//split into 2 groups:
//01101100 -> [010101] [1101100] [last index] [position]
//one piece gets 32 possible actions -> 32 bytes for one piece
//32 pieces for one player -> 32x32 -> 1024 bytes for one palyer

//display vars
LiquidCrystal_I2C lcd(0x27, 16, 2);


//Variables used for the not digital board

bool MagnetAn = false;
unsigned short xPosition;
unsigned short yPosition;


//board is defined as binary instead of array to save on memory
//may be changed to a 2 bit pieces to store information for up to 4 states:
//0b00 -> Empty Space
//0b01 -> Player 1 piece
//0b10 -> Player 2 piece
//0b11 -> To be used

//This uses the and (&) function to get specific information, board & 0b01100 extracts only the information for the second piece

uint8_t Board[16];
/* board representation as grid
    2 0 2 0 2 0 2 0
    0 2 0 2 0 2 0 2
    0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0
    1 0 1 0 1 0 1 0
->  0 1 0 1 0 1 0 1
*/

//board size
unsigned short size = 8;


//Action List for one Piece
//if the action is negative, the absolute value of that position gets removed from the game
uint8_t Actions[57];

//Action List for all pieces
uint8_t AllActions[684];

//list pointers for said lists
short act_pos;
short avAct_pos;
short newPiece_pos;
short addedFuncCnt;

//variables for general use
int boardX;
int boardY;
uint8_t boardChar[2];
int boardPos;
const short debounceTime = 50;
bool enterVal;
unsigned long passedTime = 0;
short EndPosition;
short pieceMove;
short _xPos;
short _yPos;
char text[16];


//selected Positions
short selActPos = 0;

//true: Player 1, false: Player 2
bool PlayerTurn;

int bitPicker(uint8_t _size){
  if(_size == 0){
    return 0;
  }
  int b = 1;
  for(uint8_t a = 0; a < _size-1; a++){
    b = (b<<1) +1;
  }
  return b;
}

void visArray(uint8_t* arr, short size){
  Serial.print("{");
  Serial.print(arr[0]);
  for(short i = 1; i < size; i++){
    Serial.print(",");
    Serial.print(arr[i]);
  }
  Serial.println("}");
}

void resetAct(){
  act_pos = 0;
  for(int _byte = 0; _byte < 56; _byte++){
    Actions[_byte] = 0;
  }
}

void AddPosAct(short xPos, short yPos, uint8_t lastIndex){
  short bitIndex = (act_pos * POS_BIT)%8;
  short byteIndex = ((act_pos * POS_BIT)-bitIndex)/8;
  uint8_t bytesNeeded = ceil((POS_BIT+bitIndex+7)/8);
  getPos(xPos,yPos);
  short concatinatedNum = ((lastIndex + 1) << 6) + boardPos;
  short secBitIndex = 0;
  for(int _byte = 0;_byte < bytesNeeded;_byte++){
    short necBits = min(POS_BIT - secBitIndex,8);
    short bitPos = bitPicker(necBits - bitIndex) << (POS_BIT - necBits + bitIndex- secBitIndex);
    short secBitPos = bitPicker(bitIndex) << (8-bitIndex);
    short shift = (POS_BIT - secBitIndex) + (bitIndex - 8);
    short byteMask;
    if(shift < 0){
      byteMask = (concatinatedNum & bitPos) << abs(shift);
    }
    else{
      byteMask = (concatinatedNum & bitPos) >> shift;
    }
    Actions[byteIndex + _byte] = (Actions[byteIndex + _byte] & secBitPos) | byteMask;
    secBitIndex += necBits - bitIndex;
    bitIndex = 0;
  }
  act_pos += 1;
}
void addActionToAllAction(uint8_t pos){
  AllActions[avAct_pos] = pos;
  for(uint8_t _byte = 0; _byte < ACT_BYTE-1; _byte++){
    AllActions[_byte + avAct_pos+1] = Actions[_byte];
  }
  avAct_pos += ACT_BYTE;
}
void resetAllAct(){
  avAct_pos = 0;
  for(int _byte = 0; _byte < 684; _byte++){
    AllActions[_byte] = 0;
  }
}

//assign values to the destination list from a source
void assignValues(uint8_t* dest, const uint8_t* src, uint8_t _size) {
  for (uint8_t i = 0; i < _size; i++) {
    dest[i] = src[i];
  }
}

void resetDigitalBoard(){
  const uint8_t tempBoard[16]  = {0x44,0x44,0x11,0x11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x88,0x88,0x22,0x22};
  assignValues(Board, tempBoard, 16);
  act_pos = 0;
  avAct_pos = 0;
}

void resetBoard(){
  MoveX(-xPosition);
  MoveY(-yPosition);
  MagnetSwitch(false);
}

//gets the position as a number and coordinate
void getPosCoor(int pos){
  boardX = pos % size;
  boardY = (pos - boardX) / size;
}

void getPos(int posX, int posY){
  boardPos = posX + posY * size;
}

void getPosChar(int pos){
  getPosCoor(pos);
  boardChar[0] = boardY + 65;
  boardChar[1] = boardX + 49;
}

uint8_t getPiece(uint8_t pos){
  uint8_t bitPos = pos % 4;
  return (Board[(pos-bitPos)/4] & (3 << (6-bitPos*2))) >> (6-bitPos*2);
}


//Checks wether a given position is within the boards boundaries
bool CheckPos(int posX, int posY){
  return !(posX < 0 || posX > 7 || posY < 0 || posY > 7);
}

void getActions(uint8_t pos, uint8_t lastIndex,uint8_t Player){
  getPosCoor(pos);
  if(Player == 1){
    getPosCoor(pos);
    if(CheckPos(boardX+1,boardY+1)){
      getPos(boardX+1,boardY+1);
      uint8_t piece = getPiece(boardPos);
      if(piece == Player){
        return;
      }
      else if(piece == 0){
        AddPosAct(boardX+1,boardY+1,lastIndex);
      }
      else if(piece == (Player%2)+1){
        getPos(boardX+2,boardY+2);
        if(getPiece(boardPos) == 0){
          AddPosAct(boardX+2,boardY+2,lastIndex);
          getActions(boardPos, act_pos-1, Player);
        }
      }
    }
    getPosCoor(pos);
    if(CheckPos(boardX-1,boardY+1)){
      getPos(boardX-1,boardY+1);
      uint8_t piece = getPiece(boardPos);
      if(piece == Player){
        return;
      }
      else if(piece == 0){
        AddPosAct(boardX-1,boardY+1,lastIndex);
      }
      else if(piece == (Player%2)+1){
        getPos(boardX-2,boardY+2);
        if(getPiece(boardPos) == 0){
          AddPosAct(boardX-2,boardY+2,lastIndex);
          getActions(boardPos, act_pos-1, Player);
        }
      }
    }
  }
  else{
    getPosCoor(pos);
    if(CheckPos(boardX+1,boardY-1)){
      getPos(boardX+1,boardY-1);
      uint8_t piece = getPiece(boardPos);
      if(piece == Player){
        return;
      }
      else if(piece == 0){
        AddPosAct(boardX+1,boardY-1,lastIndex);
      }
      else if(piece == (Player%2)+1){
        getPos(boardX+2,boardY-2);
        if(getPiece(boardPos) == 0){
          AddPosAct(boardX+2,boardY-2,lastIndex);
          getActions(boardPos, act_pos-1, Player);
        }
      }
    }
    getPosCoor(pos);
    if(CheckPos(boardX-1,boardY-1)){
      getPos(boardX-1,boardY-1);
      uint8_t piece = getPiece(boardPos);
      if(piece == Player){
        return;
      }
      else if(piece == 0){
        AddPosAct(boardX-1,boardY-1,lastIndex);
      }
      else if(piece == (Player%2)+1){
        getPos(boardX-2,boardY-2);
        if(getPiece(boardPos) == 0){
          AddPosAct(boardX-2,boardY-2,lastIndex);
          getActions(boardPos, act_pos-1, Player);
        }
      }
    }
  }
}

void getAllActions(uint8_t Player){
  for(uint8_t pos = 0; pos < 64; pos++){
    Serial.print("Pos: ");Serial.println(pos);
    uint8_t piece = getPiece(pos);
    Serial.print("Piece: ");Serial.println(piece);
    if(piece == Player){
      Serial.println("found Piece");
      resetAct();
      getActions(pos,0xFE,Player);
      addActionToAllAction(pos);
    }
  }
}

//Switches the magnet on and off
void MagnetSwitch(bool setting){
  if(setting){
    digitalWrite(5,HIGH);
  }
  else{
    digitalWrite(5,LOW);
  }
  MagnetAn = setting;
}


void IncSelPos(){
  if(enterVal == false){
    if(passedTime - millis() > debounceTime){
      passedTime = millis();
      selActPos += 1;
    }
  }
}

void DecSelPos(){
  if(enterVal == false){
    if(passedTime - millis() > debounceTime){
      passedTime = millis();
      selActPos -= 1;
      if(selActPos < 0){
        selActPos = 0;
      }
    }
  }
}

//setup
void setup() {
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(2), IncSelPos, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), DecSelPos, FALLING);

  lcd.init();
  
  resetBoard();
  resetDigitalBoard();

  PlayerTurn = true;

  Serial.begin(9600);
  Serial.println("");
  resetDigitalBoard();
}

//Moves along the X-Axis on the real board
void MoveX(int mov)
{
  xPosition -= mov;
  int _mov = abs(mov);
  if(_mov == mov){
    digitalWrite(8,HIGH);
    digitalWrite(7,LOW);
  }
  else{
    digitalWrite(7,HIGH);
    digitalWrite(8,LOW);
  }
  delay(CM*4*_mov);
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);
}

//Moves along the Y-Axis on the real board
void MoveY(int mov)
{
  yPosition -= mov;
  int _mov = abs(mov);
  if(_mov == mov){
    digitalWrite(13,HIGH);
    digitalWrite(12,LOW);
  }
  else{
    digitalWrite(12,HIGH);
    digitalWrite(13,LOW);
  }
  delay(CM*4*_mov);
  digitalWrite(12,LOW);
  digitalWrite(13,LOW);
}

void MoveTo(short POSX, short POSY){
  MoveX(POSX - xPosition);
  MoveY(POSY - yPosition);
}

void getActionIntoLabel(short pos){
  const char tempArray[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  short selectedPos = pos;
  short selPosInt = pos;
  int info;
  while(selPosInt > 0){
    uint8_t posBit = (selectedPos * POS_BIT) % 8;
    uint8_t posByte = (selectedPos - posBit) / 8;
    uint8_t neededBytes = (posBit + POS_BIT) / 8 +1;
    uint8_t bitPosition = 0;
    info = 0;
    for(uint8_t _byte = 0; _byte < neededBytes; _byte++){
      uint8_t _bits = min(8, POS_BIT - bitPosition);
      short bitMask = bitPicker(_bits);
      bitPosition += _bits;
      info += (AllActions[posByte + _byte] & bitMask) < POS_BIT - bitPosition;
    }
    if(info != 0){
      selPosInt -= 1;
    }
    selectedPos += 1;
  }
  short selectedChunk = (selectedPos * POS_BIT) / (ACT_BYTE+1);
  for (uint8_t _byte = 0; _byte < ACT_BYTE+1; _byte++){
    Actions[_byte] = AllActions[selectedChunk * (ACT_BYTE+1) + _byte];
  }
  short posMask = bitPicker(6);
  short replyMask = bitPicker(8) < 6;
  getPosChar(posMask & info);
  short reply = ((info & replyMask) > 6);
  short charBitPos = 0;
  text[0] = boardChar[0];
  text[1] = boardChar[1];
  text[2] = 0x2D;
  while(reply != 255){
    getPosChar(posMask & info);
    reply = ((Actions[reply] & replyMask) > 6);
    text[charBitPos * 3] = boardChar[0];
    text[charBitPos * 3 + 1] = boardChar[1];
    text[charBitPos * 3 + 2] = 0x2D;
  }
}

void getActionToLcd(short pos){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Aktion: ");
  getActionIntoLabel(pos);
  lcd.print(text);
}

void getUserInput(){
  enterVal = false;
  while(enterVal == false){
    if(digitalRead(4) == LOW){
      enterVal = true;
      getActionToLcd(selActPos);
    }
  }
}

void getPieceMovement(){
  short selectedPos = selActPos;
  short selPosInt = selActPos;
  int info;
  while(selPosInt > 0){
    uint8_t posBit = (selectedPos * POS_BIT) % 8;
    uint8_t posByte = (selectedPos - posBit) / 8;
    uint8_t neededBytes = (posBit + POS_BIT) / 8 +1;
    uint8_t bitPosition = 0;
    info = 0;
    for(uint8_t _byte = 0; _byte < neededBytes; _byte++){
      uint8_t _bits = min(8, POS_BIT - bitPosition);
      short bitMask = bitPicker(_bits);
      bitPosition += _bits;
      info += (AllActions[posByte + _byte] & bitMask) < POS_BIT - bitPosition;
    }
    if(info != 0){
      selPosInt -= 1;
    }
    selectedPos += 1;
  }
  short selectedChunk = (selectedPos * POS_BIT) / (ACT_BYTE+1);
  for (uint8_t _byte = 0; _byte < ACT_BYTE+1; _byte++){
    Actions[_byte] = AllActions[selectedChunk * (ACT_BYTE+1) + _byte];
  }
  pieceMove = Actions[0];
  EndPosition = info & 0b111111;
}

void MoveToPos(short endPos){
  MagnetSwitch(false);
  MoveX(_xPos - xPosition);
  MoveY(_yPos - yPosition);
  MagnetSwitch(true);
  getPosCoor(endPos);
  short xDif = boardX - _xPos;
  short yDif = boardY - _yPos;

  while(xDif != 0 or yDif != 0){
    if(xDif == 0){
      if(xPosition == 0){
        MoveX(1);
        xDif = -1;
      }
      else{
        MoveX(-1);
        xDif = 1;
      }
    }
    else{
      if(xDif < 0){
        MoveX(-1);
        xDif += 1;
      }
      else{
        MoveX(1);
        xDif -= 1;
      }
    }
    if(yDif < 0){
      MoveY(-1);
      yDif += 1;
    }
    else{
      MoveY(1);
      yDif -= 1;
    }
  }
  MagnetSwitch(false);
}

void changePosBoard(short pos, uint8_t piece){
  uint8_t byteInArray = pos / 4;
  uint8_t bitInArray = (pos % 4)*2;
  uint8_t BoardMask = bitPicker(bitInArray) < (8-bitInArray) + bitPicker(6-bitInArray);

  Board[byteInArray] = (Board[byteInArray] & BoardMask) | (bitPicker(2) < (6-bitInArray));
}

void MovePieces(){
  getPieceMovement();
  getPosCoor(pieceMove);
  MoveTo(boardX,boardY);
  short _xPos = boardX;
  short _yPos = boardY;
  changePosBoard(pieceMove, 0);
  changePosBoard(EndPosition, PlayerTurn + 1);

  MoveToPos(EndPosition);
}

void GameLoop(){
  Serial.println("Calculating possible actions...");
  getAllActions(PlayerTurn + 1);
  Serial.println("Done.");
  Serial.println("Waiting for User Input...");
  getUserInput();
  Serial.println("Done.");
  Serial.println("Moving Pieces...");
  MovePieces();

  PlayerTurn = !PlayerTurn;
  Serial.println("Other Players Turn!");
}

//loop
//8000000080000000800100
void loop() {
  GameLoop();
}
