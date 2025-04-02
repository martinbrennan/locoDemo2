/********************************************************
	main.c
	
	app_main is the entry point
	setup initialises the hardware and starts various services including loco
	loop contains code in the main loop called every 100ms	
	
*********************************************************/


#define DISPLAYENABLE 0

#include <loco.h>
#include "audio.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include <ctype.h>
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

/***********************************************************
 Wifi
************************************************************/

char *getLocalIp ();

int connected = 0;
int newIpFlag = 0;
int wifiDisconnectedFlag = 0;
int locoStarted = 0;

// Timezone and NTP server
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0; // Adjust for your timezone
const int daylightOffset_sec = 0; // Adjust for daylight savings if applicable


#define MYSSID "VM2984501"
#define MYPASSWORD "hmdw7HwJgdgh"

void setLocalIp(unsigned long ipAddr) {

  sprintf(getLocalIp(), "%d.%d.%d.%d", (int)ipAddr & 0xFF, (int)(ipAddr >> 8) & 0xFF,
          (int)(ipAddr >> 16) & 0xFF, (int)(ipAddr >> 24) & 0xFF);
}

static void wifiEventHandler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {

	printf ("wifiEventHandler\n");

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
	
	if (connected){			 
		connected = 0;
		getLocalIp()[0] = 0;
		wifiDisconnectedFlag = 1;
		printf ("\nWIFI DISCONNECTED\n\n");
	}    
    esp_wifi_connect();
    
    
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

    setLocalIp((unsigned long)event->ip_info.ip.addr);

    printf ("WIFI CONNECTED localIp = %s\n", getLocalIp());
    
    connected = 1;
    newIpFlag = 1;
  }
}


wifi_config_t wifiConfig = {
    .sta =
        {
            .listen_interval = 6,
            .channel = 2,
        },
};

void startWifiSta() {
  printf ("startWIfi () %s %s\n", MYSSID, MYPASSWORD);
  esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_STA);
  memset(wifiConfig.sta.ssid, 0, sizeof(wifiConfig.sta.ssid));
  memset(wifiConfig.sta.password, 0, sizeof(wifiConfig.sta.password));
  strcpy((char *)wifiConfig.sta.ssid, MYSSID);
  strcpy((char *)wifiConfig.sta.password, MYPASSWORD);
  esp_wifi_set_config(WIFI_IF_STA, &wifiConfig);
  esp_wifi_start();
}

void startWifiSta2() {
  printf("startWIfiSta2 () %s %s\n", MYSSID, MYPASSWORD);
  esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_STA);
  memset(wifiConfig.sta.ssid, 0, sizeof(wifiConfig.sta.ssid));
  memset(wifiConfig.sta.password, 0, sizeof(wifiConfig.sta.password));
  strcpy((char *)wifiConfig.sta.ssid, MYSSID);
  strcpy((char *)wifiConfig.sta.password, MYPASSWORD);
  esp_wifi_set_config(WIFI_IF_STA, &wifiConfig);
  esp_wifi_start();
}


void initialiseWifi() {

  getLocalIp()[0] = 0;

  /*init wifi as sta and set power save mode*/
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
  assert(ap_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &wifiEventHandler, NULL, NULL));
   
    startWifiSta2();
}

void locoCallback(int e) {
//  printf("locoCallback () got event %d\n", e);
  if (e == AUTHORISED) {

    printf("New Credentials %s\n", getCredentials());
    FILE *cFile = fopen("/spiffs/credentials", "wb");
    fprintf(cFile, "%s", getCredentials());
    fclose(cFile);

    printf("New User Name %s\n", getUsername());
    cFile = fopen("/spiffs/username", "wb");
    fprintf(cFile, "%s", getUsername());
    fclose(cFile);

    printf("Saved\n");
  }
 

}

#include <esp_netif_sntp.h>
#include <esp_sntp.h>

void gotTime (struct timeval *tv){
	printf ("gotTime()\n");
}	

void configureSntp (){
	
	esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG ("pool.ntp.org");
	config.start = false;
	config.sync_cb = gotTime;
	
	esp_netif_sntp_init (&config);

}

void initSntp (){		
	esp_netif_sntp_start ();
}

void doUI2 (){

  if (newIpFlag) {
	  
	printf ("newIPFlag\n");
    newIpFlag = 0;

	configureSntp ();
	initSntp (); 

    startLoco ();
    
	locoStarted = 1; 
               
  }
  if (wifiDisconnectedFlag){
	printf ("wifiDisconnectedFlag\n");
	wifiDisconnectedFlag = 0;

	stopLoco ();
	
	esp_netif_sntp_deinit ();	
  }	
}

/***********************************************************
 LED & utilities
************************************************************/

void memDebug();

const int ledPin = 11;  // GPIO 11

void initLed() {
  // zero-initialize the config structure.
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set, GPIO20
  io_conf.pin_bit_mask = 1ULL << ledPin;
  // disable pull-down mode
  io_conf.pull_down_en = 0;
  // disable pull-up mode
  io_conf.pull_up_en = 0;
  // configure GPIO with the given settings
  gpio_config(&io_conf);
  gpio_set_level(ledPin, 0);
}

void ledOn() { gpio_set_level(ledPin, 1); }

void ledOff() { gpio_set_level(ledPin, 0); }


char localIp[20];

	
char *getLocalIp() { return localIp; }

void memDebugB () {

  int l = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  int m = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  printf ("Largest block %d/%d\n", l, m);

}

void doText(unsigned char *s, int l) {

  int n;
  printf(" ");
  int m = l > 8 ? 8 : l;
  for (n = 0; n < m; n++) {
    if (isprint(*s))
      printf("%c", *s);
    else
      printf(".");
    s++;
  }
  printf("\n");
}

void cdump(unsigned char *buf, int len) {

  int n = 1;
  unsigned char *t = buf;
  unsigned tl = len;
  int a = 0;
  while (len--) {
    if (n % 8 == 1) {
      printf("%04X ", a);
      a += 8;
    }
    printf("%02x ", *buf++);
    if (!(n++ % 8)) {
      doText(t, tl);
      t = buf;
      tl -= 8;
    }
  }
  if ((n - 1) % 8)
    doText(t, tl);
}


/***********************************************************
 Setup
************************************************************/

void setup() {


  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  printf("NVS done %lld\n", millis());

  initLed();

  printf("about to call locoAudioInit()\n");

  locoAudioInit();

  printf("about to call startAudioThread()\n");

  startAudioThread();

  printf("startAudioThread() done\n");

//  initSDDetect();

  initSpiffs();

  setVolume(95);

  initialiseWifi();

  memDebugB();

  initLoco((PFVN)locoCallback, "Beauty");

  printf("Setup done\n");

  memDebugB();
}

/***********************************************************
 Main Loop
************************************************************/

char lbuf[100];
int lblen = 0;

int isSeparator(char c) {
  if (isspace(c))
	return 1;
  if (c == ',')
	return 1;
  return 0;
}

void parseCli(char *b, char **arg0, char **arg1, char **arg2) {

  *arg0 = "";
  *arg1 = "";
  *arg2 = "";

  while (*b && isSeparator(*b))
	b++; // skip leftover CRLF
  if (!*b)
	return;
  *arg0 = b;

  while (*b && !isSeparator(*b))
	b++; // skip arg0
  if (!*b)
	return;
  *b++ = 0; // terminate with a zero

  while (*b && isSeparator(*b))
	b++; // skip first separator
  if (!*b)
	return;
  *arg1 = b;

  while (*b && !isSeparator(*b))
	b++; // skip arg1
  if (!*b)
	return;
  *b++ = 0; // terminate with a zero

  while (*b && isSeparator(*b))
	b++; // skip second separator
  if (!*b)
	return;
  *arg2 = b;
}

void memDebug() {

	
  int l = heap_caps_get_total_size(MALLOC_CAP_8BIT);
  printf("Heap size %d\n", l);

  l = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  printf("SPIRAM size %d\n", l);

  l = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  printf("Largest block %d\n", l);

  l = heap_caps_get_total_size(MALLOC_CAP_DMA);
  printf("DMA size %d\n", l);
  l = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  printf("Largest block %d\n", l);

  l = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  printf("SPIRAM size %d\n", l);
  l = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  printf("Largest block %d\n", l);

  l = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
  printf("Internal size %d\n", l);
  l = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  int m = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  printf("Largest block %d/%d\n", l, m);  
    
}



void exCli() {

  char *arg0, *arg1, *arg2;
  lbuf[lblen] = 0;

  parseCli(lbuf, &arg0, &arg1, &arg2);

  if (!strcasecmp(arg0, "mem")) {
    lmemDebug();
  }
  if (!strcasecmp(arg0, "stop")) {
    stopPlay();
  }
  if (!strcasecmp(arg0, "next")) {
    startNext();
  }
  if (!strcasecmp(arg0, "prev")) {
    startPrev();
  }
  if (!strcasecmp(arg0, "resume")) {
    doResume();
  }
  if (!strcasecmp(arg0, "pause")) {
    doPause();
  }
  if (!strcasecmp(arg0, "sd")) {
    printf("sdDetect = %d\n", getSDDetect());
  }
  if (!strcasecmp(arg0, "mount")) {
    mountSd(0);
  } else if (!strcasecmp(arg0, "tt")) {
    toggleTestTone();
  } else if (!strcasecmp(arg0, "verb")) {
    int level = 0;
    sscanf(arg1, "%d", &level);
    locoVerbosity(level);
  } else if (!strcasecmp(arg0, "verify")) {
    verify();
  } else if (!strcasecmp(arg0, "restart")) {
    reconnectTest(0);
  } else if (!strcasecmp(arg0, "deb")) {
    int n = 0;
    sscanf(arg1, "%d", &n);
    locoDebug(n);
  } else if (!strcasecmp(arg0, "ver")) {
    printf("getVersion () = %s\n", getVersion(0));
  } else if (!strcasecmp(arg0, "format")) {
//    format_spiffs();
  } else if (!strcasecmp(arg0, "ssr")) {

    int index;
    sscanf(arg1, "%d", &index);
    startSearchResult(index);
  } else if (!strcasecmp(arg0, "sock")) {
    stopWebsocket();
  } else if (!strcasecmp(arg0, "life")) {
    printfTokenLife();
  } else if (!strcasecmp(arg0, "newt")) {
    refreshTokenNow();
  } else if (!strcasecmp(arg0, "togw")) {
    toggleWebsocket();
  } else if (!strcasecmp(arg0, "codec")) {
    codecRestart(95);
  } else if (!strcasecmp(arg0, "track")) {
    int track = 0;
    sscanf(arg1, "%d", &track);
	startTrackN (track);
  } else if (!strcasecmp(arg0, "status")) {
    printf ("isActive %d StateIsPlaying %d StateIsPaused %d\n",getIsActive(),getStateIsPlaying(),getStateIsPaused());
  } 
  

  lblen = 0;
}


void doCli() {

  int c = getchar();
  if (c != -1) {
	putchar(c);
	if (c == '\n')
  	putchar('\n');
	//      	printf ("%x\n",c);
  if (c == '\n')
  	exCli();
	else
  	lbuf[lblen++] = c;
	if (lblen >= 100)
  	lblen = 0;
  }
}


int n = 0;


void loop() {
  int c;
  int e;

  if (getStateIsPlaying() && !getStateIsPaused()) {
    if (n & 0x1)			// fast
      ledOn();
    else
      ledOff();
  } else {
    if ((n & 0xF) == 0xF)	// slow
      ledOn();
    else
      ledOff();
  }


  doUI2 ();

  doCli();
//  sdPoll();


  n = n + 1;
}


void app_main () {
	setup ();
	while (1){
		loop();
		vTaskDelay (pdMS_TO_TICKS(100));
	}
}


		
