/********************************************************

loco.h


*********************************************************/


#include <esp_http_server.h>
#include "cJSON.h"

#ifdef __cplusplus
 extern "C" {
#endif

// formerly types.h

typedef uint16_t u16;
typedef int16_t s16;
typedef uint8_t u8;
typedef int (*PFI)();
typedef void (*PFV)();
typedef char *(*PFC)();
typedef void (*PFVN)(int);

void initLoco (PFVN cb, char *name);
void startLoco ();
void stopLoco ();

char * getAccessToken ();
char *getPlayingTrackName ();
char *getPlayingAlbumName ();
char *getPlayingArtistName ();
char *getPlayingArtUrl ();
void locoVerbosity (int level);
void setName (char *newname);
char *getNextArtUrl ();
void lockI2c ();	
void unlockI2c ();
void lockHttps ();
void unlockHttps ();
char *getVersion (int n);
void stopPlay ();
void startNext ();
void startPrev ();
void startTrackN (int track);
void doResume ();
void doPause ();
void doReset ();
int getIsActive();
void playPause();
void playUri(char *uri);
int isDealerConnected();

int verify ();
int program ();
int do_i2cdetect_cmd();
char *getCredentials();
char *getUsername();

int getCurrentTrackIndex();
int getTrackCount ();

cJSON *vTunerSearch (char *text);
cJSON *vTunerTop ();
cJSON *vTunerBack ();
cJSON *vTunerItem (int index);

int startSearchResult(int n);
int startDirect(char *name, char *url, char *logo);
int startDirect2 (char *name, char *url, char *logo);
void restartCurrentRadio();

void locoDebug (int n);

void selectSource (int source);
int getSource ();
int isRadioSource ();
int isRadioPlaying ();
int isSpotifySource ();

int streamGet(char *url, char *buffer, int max);
char *getCurrentStationName ();
char *getCurrentStationLogo ();
char *getCurrentStationUrl ();

int getStateIsPaused();
int getStateIsPlaying();

int getAdfSamples (unsigned char *buf,int count);
char *getBreadcrumbs ();
int isApResolved();
char *getProgressString ();
char *getDurationString ();
int connectWithAuth(char *auth, char *user);
void startPlayingCurrentTrack ();


typedef enum {
	CONNECT,
	DISCONNECT,
	ACTIVE,
	INACTIVE,
	TRANSPORT,
	AUTHORISED,
	REFRESH,
	NEWART,	
} locoEvent;

#define SPOTIFYSOURCE 0
#define RADIOSOURCE 1
#define MAXNAME 200
#define STACKSIZE 5120

// WARNING temporary

void reconnectTest (int play);
void lmemDebug();
void refreshTokenNow();
void printfTokenLife ();
void stopWebsocket();
void toggleWebsocket();
void reAuth();


#ifdef __cplusplus
}
#endif
