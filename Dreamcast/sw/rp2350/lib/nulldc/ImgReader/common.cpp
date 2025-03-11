#include "common.h"
#include "gdi.h"

#include <memory.h>

uint32_t NullDriveDiscType;

Disc*(*drivers[])(char* path)=
{
	gdi_parse,
	0
};

uint8_t q_subchannel[96];

#define SETTING_SHOULD_PATCH_REGION (1)

void PatchRegion_0(uint8_t* sector,int size) {
	if (SETTING_SHOULD_PATCH_REGION==0) {
		return;
	}

	uint8_t* usersect=sector;

	if (size!=2048)
	{
		printf("PatchRegion_0 -> sector size %d , skiping patch\n",size);
	}

	//patch meta info
	uint8_t* p_area_symbol=&usersect[0x30];
	memcpy(p_area_symbol,"JUE        ",8);
}

void PatchRegion_6(uint8_t* sector,int size) {
	if (SETTING_SHOULD_PATCH_REGION==0) {
		return;
	}

	uint8_t* usersect=sector;

	if (size!=2048)
	{
		printf("PatchRegion_6 -> sector size %d , skiping patch\n",size);
	}

	//patch area symbols
	uint8_t* p_area_text=&usersect[0x700];
	memcpy(&p_area_text[4],"For JAPAN,TAIWAN,PHILIPINES.",28);
	memcpy(&p_area_text[4 + 32],"For USA and CANADA.         ",28);
	memcpy(&p_area_text[4 + 32 + 32],"For EUROPE.                 ",28);
}

bool ConvertSector(uint8_t* in_buff , uint8_t* out_buff , int from , int to,int sector)
{
	//get subchannel data, if any
	if (from==2448)
	{
		memcpy(q_subchannel,in_buff+2352,96);
		from-=96;
	}
	//if no convertion
	if (to==from)
	{
		memcpy(out_buff,in_buff,to);
		return true;
	}
	switch (to)
	{
	case 2340:
		{
			verify((from==2352));
			memcpy(out_buff,&in_buff[12],2340);
		}
		break;
	case 2328:
		{
			verify((from==2352));
			memcpy(out_buff,&in_buff[24],2328);
		}
		break;
	case 2336:
		verify(from>=2336);
		verify((from==2352));
		memcpy(out_buff,&in_buff[0x10],2336);
		break;
	case 2048:
		{
			verify(from>=2048);
			verify((from==2448) || (from==2352) || (from==2336));
			if ((from == 2352) || (from == 2448))
			{
				if (in_buff[15]==1)
				{
					memcpy(out_buff,&in_buff[0x10],2048); //0x10 -> mode1
				}
				else
					memcpy(out_buff,&in_buff[0x18],2048); //0x18 -> mode2 (all forms ?)
			}
			else
				memcpy(out_buff,&in_buff[0x8],2048);	//hmm only possible on mode2.Skip the mode2 header
		}
		break;
	case 2352:
		//if (from >= 2352)
		{
			memcpy(out_buff,&in_buff[0],2352);
		}
		break;
	default :
		printf("Sector convertion from %d to %d not supported \n", from , to);
		break;
	}

	return true;
}

//
//convert our nice toc struct to dc's native one :)

uint32_t CreateTrackInfo(uint32_t ctrl,uint32_t addr,uint32_t fad)
{
	uint8_t p[4];
	p[0]=(ctrl<<4)|(addr<<0);
	p[1]=fad>>16;
	p[2]=fad>>8;
	p[3]=fad>>0;

	return *(uint32_t*)p;
}
uint32_t CreateTrackInfo_se(uint32_t ctrl,uint32_t addr,uint32_t tracknum)
{
	uint8_t p[4];
	p[0]=(ctrl<<4)|(addr<<0);
	p[1]=tracknum;
	p[2]=0;
	p[3]=0;
	return *(uint32_t*)p;
}


void GetDriveSector(uint8_t * buff,uint32_t StartSector,uint32_t SectorCount,uint32_t secsz)
{
	if (current_disc) {
		current_disc->ReadSectors(StartSector,SectorCount,buff,secsz);
		if (current_disc->type == GdRom && StartSector==45150 && SectorCount==7)
		{
			PatchRegion_0(buff,secsz);
			PatchRegion_6(buff+2048*6,secsz);
		}
	} else {
		printf("GetDriveSector: Disc not loaded\n");
	}
}
void GetDriveToc(uint32_t* to, DiskArea area)
{
	if (!current_disc) {
		printf("GetDriveToc: Disc not loaded\n");
		return;
	}
	memset(to,0xFFFFFFFF,102*4);

	//can't get toc on the second area on discs that don't have it
	verify(area != DoubleDensity || current_disc->type == GdRom);

	//normal CDs: 1 .. tc
	//GDROM: area0 is 1 .. 2, area1 is 3 ... tc

	uint32_t first_track=1;
	uint32_t last_track=current_disc->tracks.size();
	if (area==DoubleDensity) {
		first_track = 3;
	} else if (current_disc->type==GdRom) {
		last_track = 2;
	}

	// printf("disc type=%d\n",current_disc->type);
	// printf("first_track=%d\n",first_track);
	// printf("last_track=%d\n",last_track);
	// printf("Num tracks: %d", current_disc->tracks.size());
	// printf("track0: %s\n", current_disc->tracks[0].file->filename);
	// if (current_disc->tracks.size() > 1)
	// 	printf("track1: %s\n", current_disc->tracks[1].file->filename);
	// if (current_disc->tracks.size() > 2)
	// 	printf("track2: %s\n", current_disc->tracks[2].file->filename);
	
	//Generate the TOC info

	//-1 for 1..99 0 ..98
	to[99]=CreateTrackInfo_se(current_disc->tracks[first_track-1].CTRL,current_disc->tracks[first_track-1].ADDR,first_track); 
	to[100]=CreateTrackInfo_se(current_disc->tracks[last_track-1].CTRL,current_disc->tracks[last_track-1].ADDR,last_track); 
	
	if (current_disc->type==GdRom) {
		//use smaller LEADOUT
		if (area==SingleDensity) {
			to[101]=CreateTrackInfo(current_disc->LeadOut.CTRL,current_disc->LeadOut.ADDR,13085);
		}
	}
	else {
		to[101]=CreateTrackInfo(current_disc->LeadOut.CTRL,current_disc->LeadOut.ADDR,current_disc->LeadOut.StartFAD);
	}

	for (uint32_t i=first_track-1;i<last_track;i++) {
		to[i]=CreateTrackInfo(current_disc->tracks[i].CTRL,current_disc->tracks[i].ADDR,current_disc->tracks[i].StartFAD); 
	}
}

void GetDriveSessionInfo(uint8_t* to,uint8_t session)
{
	if (!current_disc) {
		printf("GetDriveSessionInfo: Disc not loaded\n");
		return;
	}
	to[0]=2;//status , will get overwrited anyway
	to[1]=0;//0's
	
	if (session==0)
	{
		to[2]=current_disc->sessions.size();//count of sessions
		to[3]=current_disc->EndFAD>>16;//fad is sessions end
		to[4]=current_disc->EndFAD>>8;
		to[5]=current_disc->EndFAD>>0;
	}
	else
	{
		to[2]=current_disc->sessions[session-1].FirstTrack;//start track of this session
		to[3]=current_disc->sessions[session-1].StartFAD>>16;//fad is session start
		to[4]=current_disc->sessions[session-1].StartFAD>>8;
		to[5]=current_disc->sessions[session-1].StartFAD>>0;
	}
}

void printtoc(TocInfo* toc,SessionInfo* ses)
{
	printf("Sessions %d\n",ses->SessionCount);
	for (uint32_t i=0;i<ses->SessionCount;i++)
	{
		printf("Session %d: FAD %d,First Track %d\n",i+1,ses->SessionFAD[i],ses->SessionStart[i]);
		for (uint32_t t=toc->FistTrack-1;t<=toc->LastTrack;t++)
		{
			if (toc->tracks[t].Session==i+1)
			{
				printf("\tTrack %d : FAD %d CTRL %d ADR %d\n",t,toc->tracks[t].FAD,toc->tracks[t].Control,toc->tracks[t].Addr);
			}
		}
	}
	printf("Session END: FAD END %d\n",ses->SessionsEndFAD);
}

DiscType GuessDiscType(bool m1, bool m2, bool da)
{
	if ((m1==true) && (da==false) && (m2==false))
		return  CdRom;
	else if (m2)
		return  CdRom_XA;
	else if (da && m1) 
		return CdRom_Extra;
	else
		return CdRom;
}