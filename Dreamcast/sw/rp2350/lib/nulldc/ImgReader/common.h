#pragma once
#include "pico/stdlib.h"
#include "string.h"
#include "sd_utils.h"
#include <vector>
#include <vector>
#include <string>
using namespace std;

#define verify(x) if((x)==false){ printf("Verify Failed  : " #x "\n in %s -> %s : %d \n",__FUNCTION__,__FILE__,__LINE__); }

enum DiscType
{
	CdDA=0x00,
	CdRom=0x10,
	CdRom_XA=0x20,
	CdRom_Extra=0x30,
	CdRom_CDI=0x40,
	GdRom=0x80,		

	NoDisk=0x1,			//These are a bit hacky .. but work for now ...
	Open=0x2,			//tray is open :)
	Busy=0x3			//busy -> needs to be automatically done by gdhost
};

enum DiskArea
{
	SingleDensity,
	DoubleDensity
};

extern uint32_t NullDriveDiscType;
struct TocTrackInfo
{
	uint32_t FAD;	//fad , intel format
	uint8_t Control;	//cotnrol info
	uint8_t Addr;	//addr info
	uint8_t Session; //Session where teh track belongs
};
struct TocInfo
{
	//0-98 ->1-99
	TocTrackInfo tracks[99];

	uint8_t FistTrack;
	uint8_t LastTrack;

	TocTrackInfo LeadOut;	//session set to 0 on that one
};

struct SessionInfo
{
	uint32_t SessionsEndFAD;	//end of Disc (?)
	uint8_t SessionCount;	//must be at least 1
	uint32_t SessionStart[99];//start track for session
	uint32_t SessionFAD[99];	//for sessions 1-99 ;)
};

/*
Mode2 Subheader:

"1" file number for identification of nested files (0 = not interleaver.)

"2" channel number, the infantry of the various channels are selectable for playback

"3" SUBMODE byte:

7: last sector of file (EOF)
6: Real-time sector (f.Echtzeitwiedergabe without error correction)
5: Form 2 (bit = 1), form 1 (bit = 0)
4: Trigger on (depending on OS)
3: data sector (Submodebyte 3 or 2 or 1)
2: ADPCM audio sector "
1: Video-sector "
0: last sector of a record (EOR)
"4" Encoding type of audio (eg mono / stereo) and video data (in this byte data sectors is set to 0)

"5" to "8" is the repetition of "1" through "4"


RAW: 2352
MODE1:
SYNC (12) | HEAD (4) | data (2048) | edc (4) | space (8) | ecc (276)
MODE2:
SYNC (12) | HEAD (4) | sub-head (8) | sector_data (2328)
  -form1 sector_data: 
   data (2048) | edc (4) | ecc (276)

  -form2 sector_data: 
   data (2324) |edc(4)
*/

enum SectorFormat
{
	SECFMT_2352,				//full sector
	SECFMT_2048_MODE1,			//2048 user byte, form1 sector
	SECFMT_2048_MODE2_FORM1,	//2048 user bytes, form2m1 sector
	SECFMT_2336_MODE2,			//2336 user bytes, 
};

enum SubcodeFormat
{
	SUBFMT_NONE,				//No subcode info
	SUBFMT_96					//raw 96-byte subcode info
};

bool ConvertSector(uint8_t* in_buff , uint8_t* out_buff , int from , int to,int sector);

bool InitDrive(uint32_t fileflags=0);
void TermDrive();

void PatchRegion_0(uint8_t* sector,int size);
void PatchRegion_6(uint8_t* sector,int size);
void ConvToc(uint32_t* to,TocInfo* from);
void GetDriveToc(uint32_t* to,DiskArea area);
void GetDriveSector(uint8_t * buff,uint32_t StartSector,uint32_t SectorCount,uint32_t secsz);
void GetDriveSessionInfo(uint8_t* to,uint8_t session);
extern uint8_t q_subchannel[96];
extern uint8_t gdrom_read_temp_buffer[75264];

// Originally in gd_driver.h ////////////////////////////////////////
typedef void DriveRead(uint8_t * buff, uint32_t StartSector, uint32_t SectorCount, uint32_t secsz);
typedef void DriveGetToc(uint32_t* toc, DiskArea area);
typedef DiscType DriveGetType();
typedef void DriveInit();
typedef void DriveTerm();
//////////////////////////////////////////////////////////////////////

struct Session
{
	uint32_t StartFAD;			//session's start fad
	uint8_t FirstTrack;			//session's first track
};

extern FIL* track_files; // Dynamic array of track file handles
extern FATFS ddrdc_fs;
struct RawTrackFile
{
	// FIL file;
	uint8_t trackNum;
	char filename[512];
	// TODO probably need to track the current offset if the filepointer keeps dying
	int32_t offset;
	uint32_t fmt;
	bool cleanup;
	uint bytesRead = 0;

	RawTrackFile(const char* filename, uint8_t trackNum) {
		strcpy(this->filename, filename);
		this->trackNum = trackNum;
	}

	void Populate(uint32_t file_offs,uint32_t first_fad,uint32_t secfmt,const bool auto_cleanup = true)
	{
		this->offset=file_offs-first_fad*secfmt;
		this->fmt=secfmt;
		this->cleanup=auto_cleanup;
	}

	virtual void Read(uint32_t FAD,uint8_t* dst,SectorFormat* sector_type,uint8_t* subcode,SubcodeFormat* subcode_type)
	{
		//for now hackish
		if (fmt==2352)
			*sector_type=SECFMT_2352;
		else if (fmt==2048)
			*sector_type=SECFMT_2048_MODE2_FORM1;
		else if (fmt==2336)
			*sector_type=SECFMT_2336_MODE2;
		else
		{
			verify(false);
		}

		// printf("offset: %x\n", (offset+FAD*fmt));
		FRESULT fr = f_lseek(&track_files[trackNum], offset+FAD*fmt);
		if (fr != FR_OK) {
			printf("Error [(%d)(%s)] seeking file in RawTrackFile::Read()\n", fr, FRESULT_str(fr));
		}

		fr = f_read(&track_files[trackNum],dst,fmt,&bytesRead);
		if (fr != FR_OK) {
			printf("Error [(%d)(%s)] reading file in RawTrackFile::Read()\n", fr, FRESULT_str(fr));
		}

	}
	virtual ~RawTrackFile()
	{
		// printf("Destructing file...\n");
		// if (&file) {
		// 	f_close(&file);
		// }
	}
};

struct Track
{
	RawTrackFile* file;	//handler for actual IO
	uint32_t StartFAD;		//Start FAD
	uint32_t EndFAD;			//End FAD
	uint8_t CTRL;
	uint8_t ADDR;

	Track()
	{
		file = 0;
		StartFAD = 0;
		EndFAD = 0;
		CTRL = 0;
		ADDR = 0;
	}
	bool Read(uint32_t FAD,uint8_t* dst,SectorFormat* sector_type,uint8_t* subcode,SubcodeFormat* subcode_type)
	{
		if (FAD>=StartFAD && (FAD<=EndFAD || EndFAD==0) && file) {
			file->Read(FAD,dst,sector_type,subcode,subcode_type);
			return true;
		}
		else {
			printf("Common.h:162\t - Track::Read() - FAD: %u, StartFAD: %u, EndFAD: %u, File ok? %u\n", FAD, StartFAD, EndFAD, file != 0);
			return false;
		}
	}
	void Destroy() { if (file) delete file; file=0; }
};

struct Disc
{
	wstring path;
	vector<Session> sessions;	//info for sessions
	vector<Track> tracks;		//info for tracks
	Track LeadOut;				//info for lead out track (can't read from here)
	uint32_t EndFAD;					//Last valid disc sector
	DiscType type;

	//functions !
	bool ReadSector(uint32_t FAD,uint8_t* dst,SectorFormat* sector_type,uint8_t* subcode,SubcodeFormat* subcode_type)
	{
		for(size_t i = tracks.size();(i--) > 0;) {
			*subcode_type=SUBFMT_NONE;

			if (tracks[i].Read(FAD,dst,sector_type,subcode,subcode_type)) {
				return true;
			}
		}

		return false;
	}

	void ReadSectors(uint32_t FAD,uint32_t count,uint8_t* dst,uint32_t fmt)
	{
		// uint8_t temp[2352] = {0};
		
		SectorFormat secfmt;
		SubcodeFormat subfmt;		
		bool readError = false;
		// printf("ReadSectors: FAD: %u, Count: %u, FMT: %u\n", FAD, count, fmt);

		while(count)
		{	
			// uint32_t startTime = time_us_32();
			//tracks[i].Read(FAD,dst,sector_type,subcode,subcode_type)
			//file->Read(FAD,dst,sector_type,subcode,subcode_type)
			// tracks[2].file->Read(FAD,dst,&secfmt,q_subchannel,&subfmt);
    
			if (!readError && !ReadSector(FAD,gdrom_read_temp_buffer,&secfmt,q_subchannel,&subfmt))
			{				
				readError = true; //verify(false);				
				printf("ImgReader (common.h,201) - Sector read error!\n");
				printf("FAD: %u, Count:%u, FMT: %u\n",FAD,count,fmt);
			}

			//TODO: Proper sector conversions
			if (secfmt==SECFMT_2352)
			{
				uint32_t bufferOffset = 0;
				uint32_t dstOffset = 0;
				// ConvertSector(temp,dst,2352,fmt,FAD);
				// memcpy(out_buff,&in_buff[0x10],2048); //0x10 -> mode1
				do {
					memcpy(dst+dstOffset,gdrom_read_temp_buffer+0x10+bufferOffset,2048);
					bufferOffset += 2352;
					dstOffset += 2048;
				} while (bufferOffset < fmt);
			} else {
				printf("FAIL\n");
			}
			// else if (fmt == 2048 && secfmt==SECFMT_2336_MODE2)
			// 	memcpy(dst,temp+8,2048);
			// else if (fmt==2048 && (secfmt==SECFMT_2048_MODE1 || secfmt==SECFMT_2048_MODE2_FORM1 ))
			// {
			// 	memcpy(dst,temp,2048);
			// }
			// else if(!readError)
			// {
			// 	readError = true; //verify(false);
			// 	printf("ImgReader (common.h,218) - Sector conversion error!\n");
			// }

			dst+=fmt;
			FAD++;
			count--;
		}
	}
	virtual ~Disc() 
	{
		for (size_t i=0;i<tracks.size();i++)
			tracks[i].Destroy();
	};

	void FillGDSession()
	{
		Session ses;

		//session 1 : start @ track 1, and its fad
		ses.FirstTrack=1;
		ses.StartFAD=tracks[0].StartFAD;
		sessions.push_back(ses);

		//session 2 : start @ track 3, and its fad
		ses.FirstTrack=3;
		ses.StartFAD=tracks[0].StartFAD;
		sessions.push_back(ses);

		//this isn't always true for gdroms, depens on area look @ the get-toc code
		type=GdRom;
		LeadOut.ADDR=0;
		LeadOut.CTRL=0;
		LeadOut.StartFAD=549300;

		EndFAD=549300;
	}
};

extern Disc* current_disc;

DiscType GuessDiscType(bool m1, bool m2, bool da);