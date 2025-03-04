#include "gdi.h"
#include "sd_utils.h"
#include "string.h"

Disc* load_gdi(char* filename) {
	printf("Loading GDI file: %s\n", filename);

	uint32_t iso_tc;
	char line[512];
	Disc* disc = new Disc();
	uint32_t bytes_read;

	FRESULT fr;
	FIL fil;
    //FATFS fs;
    
    fr = f_mount(&ddrdc_fs, "", 1);
    if (FR_OK != fr) {
        panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return NULL;
    }

	fr = f_open(&fil, filename, FA_READ);
	
	if (FR_OK != fr && FR_EXIST != fr) {
		printf("Unable to open disc image.\n");
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
        return NULL;
    }

	// fscanf(t,"%d\r\n",&iso_tc);
	f_gets(line, sizeof(line), &fil);
	iso_tc = atoi(line);
	printf("\nGDI : %d tracks\n",iso_tc);

	char temp[512];
    char path[512];
    strcpy(path, filename);
    size_t len = strlen(filename);
    while (len > 2) {
        if (path[len] == '/') {
            break;
        }
        len--;
    }
    len++;
    char* pathptr = &path[len];
    uint32_t TRACK=0, FADS=0, CTRL=0, SSIZE=0;
    int32_t OFFSET=0;

    uint8_t numTracks = iso_tc;
    uint8_t currentTrackNum = 0;
    track_files = (FIL *)malloc(numTracks * sizeof(FIL));

    for (uint32_t i = 0; i < iso_tc; i++) {
        // Read line containing track info
        f_gets(line, sizeof(line), &fil);
        
        // Parse track info
        char* token = strtok(line, " ");
        if (token) TRACK = atoi(token);
        token = strtok(NULL, " ");
        if (token) FADS = atoi(token);
        token = strtok(NULL, " ");
        if (token) CTRL = atoi(token);
        token = strtok(NULL, " ");
        if (token) SSIZE = atoi(token);

        // Get filename - handle quoted and unquoted
        token = strtok(NULL, " \"");
        if (token) strncpy(temp, token, sizeof(temp)-1);

        // Get offset
        token = strtok(NULL, " ");
        if (token) OFFSET = atoi(token);

        printf("file[%d] \"%s\": FAD:%d, CTRL:%d, SSIZE:%d, OFFSET:%d\n",
               TRACK, temp, FADS, CTRL, SSIZE, OFFSET);
        
        Track t;
        t.ADDR = 0;
        t.StartFAD = FADS + 150;
        t.EndFAD = 0;
        t.file = 0;

        if (SSIZE != 0) {
            strcpy(pathptr, temp);

            t.file = new RawTrackFile(path, currentTrackNum);
            
            fr = f_open(&track_files[currentTrackNum++], path, FA_READ);
            if (FR_OK != fr && FR_EXIST != fr) {
                panic("gdi.cpp: f_open(%s) error: %s (%d)\n", path, FRESULT_str(fr), fr);
                return NULL;
            }
            // fr = f_lseek(&track_files[currentTrackNum], OFFSET);
            // if (fr != FR_OK) {
            //     printf("Error [(%d)(%s)] seeking file in gdi.cpp\n", fr, FRESULT_str(fr));
            // }
            // f_close(&track_files[currentTrackNum]);
            
            t.file->Populate(OFFSET, t.StartFAD, SSIZE);
        }
        disc->tracks.push_back(t);
    }

    f_close(&fil);
    disc->FillGDSession();
    return disc;
}


Disc* gdi_parse(char* file)
{
	size_t len=strlen(file);
	if (len>4) {
		if (strcmp( &file[len-4], ".gdi") == 0) {
			return load_gdi(file);
		} else {
			printf("GDI : %s is not a GDI file\n",file);
		}
	}
	return 0;
}

void iso_term()
{
}