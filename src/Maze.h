enum Orientation {ORIENTATION_UP, ORIENTATION_RIGHT, ORIENTATION_DOWN, ORIENTATION_LEFT, UNKNOWN_ORIENTATION};

#define napon 1    // 1-puna baterija  2-srednje puna baterija
#define arena 2    // 1-fiksna arena   2-mobilna arena  

//Definicije nekih kratica u programu
#define pM pinMode
#define dW digitalWrite
#define dR digitalRead
#define aW analogWrite
#define aR analogRead


//Definicije karakteristika vezanih uz motore
#define TURN_DEGREES 87.0
#define TILE_LENGTH 30
#define LEFT_MOTOR_FACTOR 0.90
#define RIGHT_MOTOR_FACTOR 1.00

// arena
#if ((arena == 1) && (napon == 1))
    #define ONE_TILE_ENCODER_STEPS 1100
    #define BLACK_MIDLE 200    // <
    #define BLACK_RIGHT 200    // <
    #define SILVER_MIDLE 650   // >
    #define SILVER_RIGHT 650   // >
    #define WHITE_MIDLE 650    // >
    #define WHITE_RIGHT 450    // <
#endif
#if ((arena == 1) && (napon == 2))
    #define ONE_TILE_ENCODER_STEPS 1200
    #define BLACK_MIDLE 200    // <
    #define BLACK_RIGHT 200    // <
    #define SILVER_MIDLE 650   // >
    #define SILVER_RIGHT 650   // >
    #define WHITE_MIDLE 650    // >
    #define WHITE_RIGHT 450    // <
#endif

// mobilne ploce
#if ((arena == 2) && (napon == 1))
    #define ONE_TILE_ENCODER_STEPS 1040
    #define BLACK_MIDLE 300    // <
    #define BLACK_RIGHT 200    // <
    #define SILVER_MIDLE 650   // >
    #define SILVER_RIGHT 650   // >
    #define WHITE_MIDLE 650    // >
    #define WHITE_RIGHT 450    // <
#endif
#if ((arena == 2) && (napon == 2))
    #define ONE_TILE_ENCODER_STEPS 1150
    #define BLACK_MIDLE 350    // <
    #define BLACK_RIGHT 250    // <
    #define SILVER_MIDLE 625   // >
    #define SILVER_RIGHT 625   // >
    #define WHITE_MIDLE 600    // >
    #define WHITE_RIGHT 450    // <
#endif

// koeficjenti lidara
// napon iznad 12.2V
#if napon == 1
    //front left
    #define FRONT_LEFT_ANG1 2.212
    #define FRONT_LEFT_MOV1 -180
    #define FRONT_LEFT_LB 220
    #define FRONT_LEFT_ANG2 2.315
    #define FRONT_LEFT_MOV2 -202
    //front middle
    #define FRONT_MIDDLE_ANG1 2.188
    #define FRONT_MIDDLE_MOV1 -176
    #define FRONT_MIDDLE_LB 220
    #define FRONT_MIDDLE_ANG2 2.11
    #define FRONT_MIDDLE_MOV2 -159
    //front right
    #define FRONT_RIGHT_ANG1 2.243
    #define FRONT_RIGHT_MOV1 -177
    #define FRONT_RIGHT_LB 215
    #define FRONT_RIGHT_ANG2 2.294
    #define FRONT_RIGHT_MOV2 -188
    //left front
    #define LEFT_FRONT_ANG1 2.151
    #define LEFT_FRONT_MOV1 -146
    #define LEFT_FRONT_LB 210
    #define LEFT_FRONT_ANG2 3.038
    #define LEFT_FRONT_MOV2 -331
    //left middle
    #define LEFT_MIDDLE_ANG1 2.176
    #define LEFT_MIDDLE_MOV1 -142
    #define LEFT_MIDDLE_LB 205
    #define LEFT_MIDDLE_ANG2 2.132
    #define LEFT_MIDDLE_MOV2 -133
    //left back
    #define LEFT_BACK_ANG1 2.292
    #define LEFT_BACK_MOV1 -194
    #define LEFT_BACK_LB 220
    #define LEFT_BACK_ANG2 2.737
    #define LEFT_BACK_MOV2 -290
    //right front
    #define RIGHT_FRONT_ANG1 2.516
    #define RIGHT_FRONT_MOV1 -222
    #define RIGHT_FRONT_LB 210
    #define RIGHT_FRONT_ANG2 2.516
    #define RIGHT_FRONT_MOV2 -222
    //right middle
    #define RIGHT_MIDDLE_ANG1 2.204
    #define RIGHT_MIDDLE_MOV1 -183
    #define RIGHT_MIDDLE_LB 210
    #define RIGHT_MIDDLE_ANG2 2.272
    #define RIGHT_MIDDLE_MOV2 -198
    //right back
    #define RIGHT_BACK_ANG1 2.041
    #define RIGHT_BACK_MOV1 -140
    #define RIGHT_BACK_LB 220
    #define RIGHT_BACK_ANG2 2.443
    #define RIGHT_BACK_MOV2 -227
    //back left
    #define BACK_LEFT_ANG1 2.339
    #define BACK_LEFT_MOV1 -215
    #define BACK_LEFT_LB 225
    #define BACK_LEFT_ANG2 2.177
    #define BACK_LEFT_MOV2 -179
    //back right
    #define BACK_RIGHT_ANG1 2.232
    #define BACK_RIGHT_MOV1 -176
    #define BACK_RIGHT_LB 215
    #define BACK_RIGHT_ANG2 2.542
    #define BACK_RIGHT_MOV2 -243

    #define FRONT_NEAR_DISTANCE 5.0
#endif

//napon ispod 12.1V
#if napon == 2
    //front left
    #define FRONT_LEFT_ANG1 0.851
    #define FRONT_LEFT_MOV1 -204
    #define FRONT_LEFT_LB 600
    #define FRONT_LEFT_ANG2 0.943
    #define FRONT_LEFT_MOV2 -258
    //front middle
    #define FRONT_MIDDLE_ANG1 0.943
    #define FRONT_MIDDLE_MOV1 -268
    #define FRONT_MIDDLE_LB 610
    #define FRONT_MIDDLE_ANG2 0.83
    #define FRONT_MIDDLE_MOV2 -198
    //front right
    #define FRONT_RIGHT_ANG1 0.9
    #define FRONT_RIGHT_MOV1 -237
    #define FRONT_RIGHT_LB 600
    #define FRONT_RIGHT_ANG2 0.882
    #define FRONT_RIGHT_MOV2 -226
    //left front
    #define LEFT_FRONT_ANG1 0.833
    #define LEFT_FRONT_MOV1 -204
    #define LEFT_FRONT_LB 610
    #define LEFT_FRONT_ANG2 1.01
    #define LEFT_FRONT_MOV2 -311
    //left middle
    #define LEFT_MIDDLE_ANG1 0.86
    #define LEFT_MIDDLE_MOV1 -211
    #define LEFT_MIDDLE_LB 610
    #define LEFT_MIDDLE_ANG2 0.84
    #define LEFT_MIDDLE_MOV2 -200
    //left back
    #define LEFT_BACK_ANG1 0.8065
    #define LEFT_BACK_MOV1 -309
    #define LEFT_BACK_LB 760
    #define LEFT_BACK_ANG2 1.24
    #define LEFT_BACK_MOV2 -636
    //right front
    #define RIGHT_FRONT_ANG1 1.111
    #define RIGHT_FRONT_MOV1 -330
    #define RIGHT_FRONT_LB 570
    #define RIGHT_FRONT_ANG2 1.111
    #define RIGHT_FRONT_MOV2 -330
    //right middle
    #define RIGHT_MIDDLE_ANG1 0.897
    #define RIGHT_MIDDLE_MOV1 -251
    #define RIGHT_MIDDLE_LB 620
    #define RIGHT_MIDDLE_ANG2 0.9375
    #define RIGHT_MIDDLE_MOV2 -277
    //right back
    #define RIGHT_BACK_ANG1 0.881
    #define RIGHT_BACK_MOV1 -230
    #define RIGHT_BACK_LB 610
    #define RIGHT_BACK_ANG2 0.9434
    #define RIGHT_BACK_MOV2 -267
    //back left
    #define BACK_LEFT_ANG1 0.8888
    #define BACK_LEFT_MOV1 -253
    #define BACK_LEFT_LB 630
    #define BACK_LEFT_ANG2 0.9375
    #define BACK_LEFT_MOV2 -283
    //back right
    #define BACK_RIGHT_ANG1 0.897
    #define BACK_RIGHT_MOV1 -247
    #define BACK_RIGHT_LB 620
    #define BACK_RIGHT_ANG2 1.0345
    #define BACK_RIGHT_MOV2 -331

    #define FRONT_NEAR_DISTANCE 5.0
#endif

/*
 *Struktura u koju je zapakirana trojka kordinata (x, y z), koristi 3 podatka tipa short int radi uklanjanja poteškoća prijašnjeg programa 
 *(negativne koordinate nisu postojale, uzrokovale bug u programu), ali i troši manje memorije od tipa podatka int
 */
struct Position {
    public:
        short int positionX;
        short int positionY;
        short int positionZ;
        
        Position() {
            this->positionX = 0;
            this->positionY = 0;
            this->positionZ = 0;
        }

        Position(short int newPositionX, short int newPositionY, short int newPositionZ) {
            this->positionX = newPositionX;
            this->positionY = newPositionY;
            this->positionZ = newPositionZ;
        }
};


/*
 *Klasa Tile osnovna je klasa koja se koristi za mapiranje labirinta. Ima 3 atributa za zapis podataka o polju. 
 *Strukturu Position (odnosno pokazivač na strukturu, time osiguravamo da postoji samo jedna struktura za svaku koordinatu i štedimo memoriju),
 *enumerator Tiletype koji sadrži informacije o tipu polja te char accesibleTiles za zapis dostupnih susjednih polja. Za taj zapis koriste se 4 
 *niža bita chara (od 8 mogućih). Najviši označava polje iznad trenutnog ("iznad" označava orijentaciju na početku vožnje, prilikom 
 *inicijalizacije robota). Ostala idu u smjeru kazaljke na satu do najnižeg bita. 1 - polje je dostupno, 0 - polje nije dostupno, postoji zid.
 */
class Tile {
    public:
        enum TileType {NORMAL, BLUE, BLACK, STAIRS, RAMP, CHECKPOINT, SPEED_BUMPS, UNKNOWN, RED, GREEN};
        enum VictimType {THERMAL, BLUE_, NONE};

    private:
        Position *tilePosition;
        TileType tileType;
        char accesibleTiles;
        VictimType victimType;

    public:
        Tile() {
            this->tilePosition = new Position(0, 0, 0);
            this->tileType = TileType::UNKNOWN;
            this->accesibleTiles = 0x00;
            this->victimType = NONE;
        }

        Tile(Position newPosition, TileType newTileType, char newAccesibleTiles, VictimType newVictimType) {
            this->tilePosition = &newPosition;
            this->tileType = newTileType;
            this->accesibleTiles = newAccesibleTiles;
            this->victimType = newVictimType;
        }

        Tile(short int newPositionX, short int newPositionY, short int newPositionZ, TileType newTileType, char newAccesibleTiles, VictimType newVictimType) {
            this->tilePosition = new Position(newPositionX, newPositionY, newPositionZ);
            this->tileType = newTileType;
            this->accesibleTiles = newAccesibleTiles;
            this->victimType = newVictimType;
        }

        Position *getPosition();
        TileType *getTileType();
        char *getAccesibleNeighbourTiles();
        VictimType *getVictimType();

        void setPosition(short int newPositionX, short int newPositionY, short int newPositionZ);
        void setTileType(TileType newTileType);
        void setAccesibleTile(Tile *accesibleTile);
        void setVictimType(VictimType newVictimType);


        bool isAccesibleNeighbourTile(short int differenceX, short int differenceY, short int differenceZ);
        bool isEqualPosition(Tile *otherTile);
        bool isEqualPosition(short int positionX, short int positionY, short int positionZ);
        short int tileTypeToWeight();
        void printTile();
};


/*
 *Pomoćna klasa u koju su zapakirani podaci o polju (instance klase Tile). Svakom objekt TileListElement pridružen je točno jedan Tile 
 *(ali jedan Tile može biti pridružen više TileListElement). Budući da je dvostruko vezana lista, svaki element liste sadrži pokazivač na 
 *PRETHODNI I SLJEDEĆI element liste (prvi -> prethodni element mu je nullptr, odnosno ništa, zadnji -> sljedeći element mu je nullptr):
 */
class TileListElement {
    private:
        TileListElement *previousElement;
        TileListElement *nextElement;
        Tile *tile;

    public:
        TileListElement() {
            this->previousElement = nullptr;
            this->nextElement = nullptr;
            this->tile = nullptr;
        }

        TileListElement(Tile *tile) {
            this->previousElement = nullptr;
            this->nextElement = nullptr;
            this->tile = tile;
        }

        TileListElement *getPreviousElement();
        TileListElement *getNextElement();
        Tile *getTile();

        void setPreviousElement(TileListElement *previousElement);
        void setNextElement(TileListElement *nextElement);
};


/*
 *Klasa koja je naša implementacija liste. Sastoji se od samo 2 pokazivača: prvog elementa liste (tzv. glava) i zadnjeg elementa liste (tzv. rep)
 */
class TilesList {
    private:
        TileListElement *firstElement;
        TileListElement *lastElement;

    public:
        TilesList() {
            this->firstElement = nullptr;
            this->lastElement = nullptr;
        }

        TileListElement *getFirstElement();
        TileListElement *getLastElement();

        void setFirstElement(TileListElement *firstElement);
        void setLastElement(TileListElement *lastElement);
        
        TileListElement *getElementByPosition(short int positionX, short int positionY, short int positionZ);
        void addElement(Tile *newTile);
        bool isEmpty();
        bool checkIfTileExists(short int positionX, short int positionY, short int positionZ);
        void printAllElements();
        bool removeElement(Tile *tile);
        void removeAllElements(); 
};

/*
 *Pomoćna klasa u koju su zapakirani podaci o polju (instance klase Tile) te udaljenosti do tog polja. 
 *Svakom objektu DistanceSetElement pridružen je točno jedan uređeni par (Tile *, short int). Također,  budući da je udaljenost od trenutno
 *promatranog polja do svakog drugog poznatog polja jedinstvena, svaki Tile se smije nalaziti samo jednom u setu (skupu). Kao i kod liste,
 *svaki element sadrži pokazivač na prethodni i sljedeći element seta.
 */
class DistanceSetElement {
    private:
        DistanceSetElement *previousElement;
        DistanceSetElement *nextElement;
        Tile *tile;
        short int distance;

    public:
        DistanceSetElement() {
            this->previousElement = nullptr;
            this->nextElement = nullptr;
            this->tile = nullptr;
            this->distance = -1;
        }

        DistanceSetElement(Tile *tile) {
            this->previousElement = nullptr;
            this->nextElement = nullptr;
            this->tile = tile;
            this->distance = -1;
        }

        DistanceSetElement *getPreviousElement();
        DistanceSetElement *getNextElement();
        Tile *getTile();
        short int getDistance();

        void setPreviousElement(DistanceSetElement *previousElement);
        void setNextElement(DistanceSetElement *nextElement);
        void setDistance(short int distance);
};


/*
 *Implementacija našeg skupa udaljenosti, sadrži pokazivač na prvi i zadnji element seta.
 */
class DistanceSet {
    private:
        DistanceSetElement *firstElement;
        DistanceSetElement *lastElement;

    public:
        DistanceSet() {
            this->firstElement = nullptr;
            this->lastElement = nullptr;
        }

        DistanceSetElement *getFirstElement();
        DistanceSetElement *getLastElement();

        void setFirstElement(DistanceSetElement *firstElement);
        void setLastElement(DistanceSetElement *lastElement);
        
        void addElement(Tile *newTile);
        DistanceSetElement *getElementByPosition(short int positionX, short int positionY, short int positionZ);
        DistanceSetElement *getElementByTile(Tile *tile);
        void setAllDistances(short int newDistance);
        bool isEmpty();
        bool setDistanceOfElement(Tile *tile, short int newDistance);
        bool checkIfTileExists(short int positionX, short int positionY, short int positionZ);
        Tile *getSmallestNeighbour(Tile *tile);
        void printAllElements();
        bool removeElement(Tile *tile); 
        void removeAllElements();
};


/*
 *Klasa koja modelira naš maze. Sadrži 2 liste (posjećenih i neposjećenih polja), skup udaljenosti od trenutnog do svih ostalih polja te pokazivač
 *na trenutno polje. Osim toga, ima podatke o početnim (uvijek trojka (0, 0, 0)) te trenutnim koordinatama robota unutar labirinta (mijenja se).
 */
class Maze {
    private: 
        TilesList *visitedTiles;
        TilesList *unknownTiles;
        DistanceSet *distanceSet;
        Tile *currentTile;
        short int startX;
        short int startY;
        short int startZ;
        short int currentX;
        short int currentY;
        short int currentZ;

    public:
        Maze() {
            this->visitedTiles = new TilesList();
            this->unknownTiles = new TilesList();
            this->distanceSet = new DistanceSet();
            this->currentTile = new Tile(0, 0, 0, Tile::TileType::NORMAL, 0x00, Tile::VictimType::NONE);
            this->startX = 0;
            this->startY = 0;
            this->startZ = 0;
            this->currentX = 0;
            this->currentY = 0;
            this->currentZ = 0;

            this->getVisitedTiles()->addElement(currentTile);
            this->getDistanceSet()->addElement(currentTile);
        }

        TilesList *getVisitedTiles();
        TilesList *getUnknownTiles();
        DistanceSet *getDistanceSet();
        Tile *getCurrentTile();
        short int getCurrentX();
        short int getCurrentY();
        short int getCurrentZ();

        void setCurrentTile(short int currentX, short int currentY, short int currentZ);
        void addUnknownTile(Tile *unknownTile);
        void addVisitedTile(Tile *visitedTile);
        void advance(Orientation orientation);

        bool existUnvisitedTile();
        
        Tile *getNextTile();
        Tile *getNextDestination();
        Orientation getOrientationToNextTile(Tile *startTile, Tile *nextTile);
        void shortestDistanceToDestination(Tile *startTile, Tile *destinationTile);
        void printMaze();
};


class Robot {
    public:
        Orientation getRobotOrientation();
        void setRobotOrientation(Orientation newOrientation);
        enum State {SUSPEND, GO_AHEAD_1_TILE, TURN_LEFT, TURN_RIGHT, DECIDE_AND_SEARCH_VICTIMS};

        float distanceFrontLeft();
        float distanceFrontMiddle();
        float distanceFrontRight();
        float distanceRightFront();
        float distanceRightMiddle();
        float distanceRightBack();
        float distanceBackRight();
        float distanceBackLeft();
        float distanceLeftFront();
        float distanceLeftMiddle();
        float distanceLeftBack();
        float compassHeading();
        bool setupHardware();
        bool isWallFront();
        bool isWallFrontNear();
        float WallFrontDistance();
        bool isWallLeft();
        bool isWallRight();
        bool isWallBack();
        void turn90Degrees(bool isLeft = false);
        int goAheadOneTile();
        void stop();
        void followIMU(float startHeading);
        void followRightLidars(float startCompass);
        void followLeftLidars(float startCompass);
        void blackTileBack();
        State getState();
        void setState(State state);
        int crnosrebrnobijelo ();
        void pauza (int duljina);
        char colorDetection(); 
        float colorRed();
        float colorOrange();
        float colorYellow();
        float colorGreen();
        float colorBlue();
        float colorViolet();
        void StartStop();
        int followingType();
        void followToNextTyle(int fType, float startCompass);
        void tileSignal (char Signal);

        void testSensors();


        

    private:
        Orientation robotOrientation;
        State state;
        void changeRobotOrientation(bool isLeft = false);
};
