/*
 * $Id: descriptors.cpp,v 1.65 2004/02/17 16:26:07 thegoodguy Exp $
 *
 * (C) 2002-2003 Andreas Oberritter <obi@tuxbox.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <cstdio>
#include <map>
#include <string>

/* libevent */
#include <eventserver.h>

#include <zapit/bouquets.h>
#include <zapit/client/zapitclient.h>
#include <zapit/descriptors.h>
#include <zapit/dvbstring.h>
#include <zapit/frontend_c.h>
#include <zapit/getservices.h>
#include <zapit/scan.h>
#include <zapit/sdt.h>
#include <zapit/pat.h>
#include <zapit/pmt.h>
#include <zapit/debug.h>
#include <zapit/zapit.h>
#include <dmx.h>
#include <math.h>

extern CBouquetManager *g_bouquetManager;
extern CZapitClient::scanType scanType;
std::string curr_chan_name;
std::string lastProviderName;
//std::string lastServiceName;
//std::map <t_channel_id, uint8_t> service_types;

extern CEventServer *eventServer;
int scan_fta_flag = 0;

void generic_descriptor(const unsigned char * const)
{
#if 0
	DBG("generic descriptor dump:");
	for (unsigned short i = 0; i < buffer[1] + 2; i++)
		printf(" %02x", buffer[i]);
	printf("\n");
#endif
}

/* 0x02 */
void video_stream_descriptor(const unsigned char * const)
{
}

/* 0x03 */
void audio_stream_descriptor(const unsigned char * const)
{
}

/* 0x04 */
void hierarchy_descriptor(const unsigned char * const)
{
}

/* 0x05 */
void registration_descriptor(const unsigned char * const)
{
}

/* 0x06 */
void data_stream_alignment_descriptor(const unsigned char * const)
{
}

/* 0x07 */
void target_background_grid_descriptor(const unsigned char * const)
{
}

/* 0x08 */
void Video_window_descriptor(const unsigned char * const)
{
}

/* 0x09 */
void CA_descriptor(const unsigned char * const buffer, uint16_t /*ca_system_id*/, uint16_t* ca_pid)
{
//	if ((((buffer[2] & 0x1F) << 8) | buffer[3]) == ca_system_id)
		*ca_pid = ((buffer[4] & 0x1F) << 8) | buffer[5];
}

/* 0x0A */
void ISO_639_language_descriptor(const unsigned char * const)
{
}

/* 0x0B */
void System_clock_descriptor(const unsigned char * const)
{
}

/* 0x0C */
void Multiplex_buffer_utilization_descriptor(const unsigned char * const)
{
}

/* 0x0D */
void Copyright_descriptor(const unsigned char * const)
{
}

/* 0x0E */
void Maximum_bitrate_descriptor(const unsigned char * const)
{
}

/* 0x0F */
void Private_data_indicator_descriptor(const unsigned char * const)
{
}

/* 0x10 */
void Smoothing_buffer_descriptor(const unsigned char * const)
{
}

/* 0x11 */
void STD_descriptor(const unsigned char * const)
{
}

/* 0x12 */
void IBP_descriptor(const unsigned char * const)
{
}

/*
 * 0x13 ... 0x1A: Defined in ISO/IEC 13818-6
 */

/* 0x1B */
void MPEG4_video_descriptor(const unsigned char * const)
{
}

/* 0x1C */
void MPEG4_audio_descriptor(const unsigned char * const)
{
}

/* 0x1D */
void IOD_descriptor(const unsigned char * const)
{
}

/* 0x1E */
void SL_descriptor(const unsigned char * const)
{
}

/* 0x1F */
void FMC_descriptor(const unsigned char * const)
{
}

/* 0x20 */
void External_ES_ID_descriptor(const unsigned char * const)
{
}

/* 0x21 */
void MuxCode_descriptor(const unsigned char * const)
{
}

/* 0x22 */
void FmxBufferSize_descriptor(const unsigned char * const)
{
}

/* 0x23 */
void MultiplexBuffer_descriptor(const unsigned char * const)
{
}

/* 0x24 */
void FlexMuxTiming_descriptor(const unsigned char * const)
{
}

/*
 * 0x25 ... 0x39:  ITU-T H.222.0 | ISO/IEC 13818-1 Reserved
 */

/* 0x40 */
void network_name_descriptor(const unsigned char * const /*buffer*/)
{
#if 0
	unsigned char tag = buffer[0];
	unsigned char len = buffer[1];
	char name[255];
	int i;

	for(i = 0; i < len; i++)
		name[i] = buffer[2+i];
	name[i] = 0;
	printf("[nit] network name: %s\n", name);
#endif
}

/* 0x41 */
void service_list_descriptor(const unsigned char * const buffer, const t_transport_stream_id transport_stream_id, const t_original_network_id original_network_id, t_satellite_position satellitePosition, freq_id_t freq)
{
	for (int i = 0; i < buffer[1]; i += 3) {
		t_service_id service_id = buffer[i + 2] << 8 | buffer[i + 3];
		t_channel_id channel_id = CREATE_CHANNEL_ID64;
		uint8_t service_type = buffer[i+4];
//printf("[service_list] type %X sid %X\n", service_type, service_id);
		if(service_type == 0x9A) service_type = 1;
		if(service_type == 0x86) service_type = 1;
		//service_types[channel_id] = service_type;
		CServiceScan::getInstance()->AddServiceType(channel_id, service_type);
	}
}

/* 0x42 */
void stuffing_descriptor(const unsigned char * const)
{
}

/* 0x43 */
int satellite_delivery_system_descriptor(const unsigned char * const buffer, t_transport_stream_id transport_stream_id, t_original_network_id original_network_id, t_satellite_position satellitePosition, freq_id_t freq)
{
	FrontendParameters feparams;
	uint8_t polarization;
	stiterator tI;
	transponder_id_t TsidOnid;
	int modulationSystem, modulationType/*, rollOff*/, fec_inner;

	if (CFrontend::getInstance()->getInfo()->type != FE_QPSK)
		return -1;

	feparams.frequency =
		(
		 ((buffer[2] >> 4)	* 100000000) +
		 ((buffer[2] & 0x0F)	* 10000000) +
		 ((buffer[3] >> 4)	* 1000000) +
		 ((buffer[3] & 0x0F)	* 100000) +
		 ((buffer[4] >> 4)	* 10000) +
		 ((buffer[4] & 0x0F)	* 1000) +
		 ((buffer[5] >> 4)	* 100) +
		 ((buffer[5] & 0x0F)	* 10)
		);

	feparams.inversion = INVERSION_AUTO;

//	rollOff = (buffer[8] >> 4) & 0x03; //alpha_0_35, alpha_0_25, alpha_0_20, alpha_auto
	modulationSystem = (buffer[8] >> 2) & 0x01; // 1= DVB_S2
	modulationType = (buffer[8]) & 0x03; // 1=QPSK, 2=M8PSK

	feparams.u.qpsk.symbol_rate =
		(
		 ((buffer[9] >> 4)	* 100000000) +
		 ((buffer[9] & 0x0F)	* 10000000) +
		 ((buffer[10] >> 4)	* 1000000) +
		 ((buffer[10] & 0x0F)	* 100000) +
		 ((buffer[11] >> 4)	* 10000) +
		 ((buffer[11] & 0x0F)	* 1000) +
		 ((buffer[12] >> 4)	* 100)
		);

	fec_inner = CFrontend::getCodeRate(buffer[12] & 0x0F, modulationSystem);
	if(modulationType == 2)
		fec_inner += 9;

	feparams.u.qpsk.fec_inner = (fe_code_rate_t) fec_inner;

	polarization = (buffer[8] >> 5) & 0x03;

	/* workarounds for braindead broadcasters (e.g. on Telstar 12 at 15.0W) */
	if (feparams.frequency >= 100000000)
		feparams.frequency /= 10;
	if (feparams.u.qpsk.symbol_rate >= 50000000)
		feparams.u.qpsk.symbol_rate /= 10;

	feparams.frequency = (int) 1000 * (int) round ((double) feparams.frequency / (double) 1000);

	freq = feparams.frequency / 1000;

	if(feparams.frequency > 15000000) {
		printf("[NIT] ******************************************* Bogus TP: freq %d SR %d fec %d pol %d\n", feparams.frequency, feparams.u.qpsk.symbol_rate, fec_inner, polarization);
		return 0;
	}
	TsidOnid = CREATE_TRANSPONDER_ID_FROM_SATELLITEPOSITION_ORIGINALNETWORK_TRANSPORTSTREAM_ID(freq, satellitePosition, original_network_id, transport_stream_id);
	CServiceScan::getInstance()->AddTransponder(TsidOnid, &feparams, polarization, true);

	return 0;
}

/* 0x44 */
int cable_delivery_system_descriptor(const unsigned char * const buffer, t_transport_stream_id transport_stream_id, t_original_network_id original_network_id, t_satellite_position satellitePosition, freq_id_t freq)
{
	transponder_id_t TsidOnid;
	if (CFrontend::getInstance()->getInfo()->type != FE_QAM)
		return -1;

	FrontendParameters feparams;

	feparams.frequency =
	(
		((buffer[2] >> 4)	* 1000000000) +
		((buffer[2] & 0x0F)	* 100000000) +
		((buffer[3] >> 4)	* 10000000) +
		((buffer[3] & 0x0F)	* 1000000) +
		((buffer[4] >> 4)	* 100000) +
		((buffer[4] & 0x0F)	* 10000) +
		((buffer[5] >> 4)	* 1000) +
		((buffer[5] & 0x0F)	* 100)
	);

	feparams.inversion = INVERSION_AUTO;

	feparams.u.qam.symbol_rate =
	(
		((buffer[9] >> 4)	* 100000000) +
		((buffer[9] & 0x0F)	* 10000000) +
		((buffer[10] >> 4)	* 1000000) +
		((buffer[10] & 0x0F)	* 100000) +
		((buffer[11] >> 4)	* 10000) +
		((buffer[11] & 0x0F)	* 1000) +
		((buffer[12] >> 4)	* 100)
	);

        if(feparams.frequency > 1000*1000)
                feparams.frequency /= 1000;

	feparams.u.qam.fec_inner = CFrontend::getCodeRate(buffer[12] & 0x0F);
	feparams.u.qam.modulation = CFrontend::getModulation(buffer[8]);
//printf("TP:: freq %X Frequency %X ID %llx\n", freq, feparams.frequency, CREATE_TRANSPONDER_ID_FROM_SATELLITEPOSITION_ORIGINALNETWORK_TRANSPORTSTREAM_ID(freq, satellitePosition, original_network_id, transport_stream_id));

        //feparams.frequency = (int) 1000 * (int) round ((double) feparams.frequency / (double) 1000);
        freq = feparams.frequency / 100;
        TsidOnid = CREATE_TRANSPONDER_ID_FROM_SATELLITEPOSITION_ORIGINALNETWORK_TRANSPORTSTREAM_ID(freq, satellitePosition, original_network_id, transport_stream_id);
        CServiceScan::getInstance()->AddTransponder(TsidOnid, &feparams, 0, true);
	return 0;
}

/* 0x45 */
void VBI_data_descriptor(const unsigned char * const)
{
}

/* 0x46 */
void VBI_teletext_descriptor(const unsigned char * const)
{
}

/* 0x47 */
void bouquet_name_descriptor(const unsigned char * const)
{
}

uint8_t fix_service_type(uint8_t type)
{
	if((type == 0x9A) || (type == 0x86) || (type==0xc3)
		|| (type==0xc5) || (type==0xc6)  ||
		(type == 0x11) || (type == 0x16) || (type == 0x19) || (type == 0x82) |
		(type == 0x87) || (type == 0xd3)  )
			return 1;
	return type;
}
bool check_blacklisted_digital_plus(const t_original_network_id onid, const t_transport_stream_id tsid)
{
	if ( (onid == 0x0001) &&
		((tsid == 0x03F0) || (tsid == 0x03F8) || (tsid == 0x0404) || (tsid == 0x0408) || (tsid == 0x040A) || (tsid == 0x040E) ||
		(tsid == 0x0412) || (tsid == 0x0416) || (tsid == 0x041A) || (tsid == 0x041E) || (tsid == 0x0420) || (tsid == 0x0422) || (tsid == 0x0424)) )
		return true;
	else
		return false;
}

void removeMultipleWhitespaces (std::string &str)
{
	size_t pos = str.find("  ");
	if(pos != std::string::npos ){
		std::string temp;
		for ( unsigned short i = 0 ; i < str.length(); i++)
			if (!(str[i] == ' ' && str[i+1] == ' '))
				temp += str[i];
		str = temp;
	}
}

bool check_blacklisted(std::string& providerName)
{
	bool in_blacklist = false;
	const char *Cyfrowy_Polsat="Cyfrowy Polsat";

	if (providerName == "CanalSat\xE9lite") {
		providerName = "CanalSat\xC3\xA9lite";
		in_blacklist = true;
	} else if (providerName == "Chambre des D\xE9" "put\xE9" "es") {
		providerName = "Chambre des D\xC3\xA9" "put\xC3\xA9" "es";
		in_blacklist = true;
	} else if (providerName == "SKY") {
		providerName = "Sky"; // well the name PREMIERE itself is not a problem
		in_blacklist = true;
	} else if(strncasecmp(providerName.c_str(),"TVN ",4)==0) {
		providerName = "TVN";
		in_blacklist = true;
	}else if (providerName == "BetaDigital"){
		in_blacklist = true;
	}else if (providerName == "Radio Maria �sterreich"){
		providerName="Radio Maria \xc3\x96sterreich";
		in_blacklist = true;
	} else if(strncasecmp(providerName.c_str(),Cyfrowy_Polsat,14)==0){
		providerName = Cyfrowy_Polsat;
		in_blacklist = true;
	}
	return in_blacklist;
}

/* 0x48 */
void service_descriptor(const unsigned char * const buffer, const t_service_id service_id, const t_transport_stream_id transport_stream_id, const t_original_network_id original_network_id, t_satellite_position satellitePosition, freq_id_t freq, bool free_ca)
{
	bool service_wr = false;
	uint8_t service_type = buffer[2];
	CZapitChannel *channel = NULL;
	bool tpchange = false;
	static transponder_id_t last_tpid = 0;
	//scrambled
	if(free_ca && scan_fta_flag){
		return;
	}

	service_type = fix_service_type(service_type);
	uint8_t real_type = service_type;

	switch ( scanType ) {
		case CZapitClient::ST_TVRADIO:
			if ( (service_type == 1 ) || (service_type == 2) )
				service_wr=true;
			break;
		case CZapitClient::ST_TV:
			if ( service_type == 1 )
				service_wr=true;
			break;
		case CZapitClient::ST_RADIO:
			if ( service_type == 2 )
				service_wr=true;
			break;
		case CZapitClient::ST_ALL:
			service_wr=true;
			break;
	}

	if ( !service_wr )
		return;

	uint8_t service_provider_name_length = buffer[3];

	std::string providerName((const char*)&(buffer[4]), service_provider_name_length);
	std::string serviceName;
//	std::string satelliteName = "unknown";
	bool in_blacklist = false;

	if (check_blacklisted(providerName)) {
		in_blacklist = true;
	}else if((check_blacklisted_digital_plus(original_network_id, transport_stream_id))){
		providerName = "Digital+";
	        in_blacklist = true;
	}

	if (in_blacklist) {
		if (((unsigned char)buffer[4 + service_provider_name_length + 1]) >= 0x20) // no encoding info
			serviceName  = CDVBString(("\x05" + std::string((const char*)&(buffer[4 + service_provider_name_length + 1]), (2 + buffer[1]) - (4 + service_provider_name_length + 1))).c_str(), (2 + buffer[1]) - (4 + service_provider_name_length + 1) + 1).getContent(); // add artificial encoding info
		else
			serviceName  = CDVBString((const char*)&(buffer[4 + service_provider_name_length + 1]), (2 + buffer[1]) - (4 + service_provider_name_length + 1)).getContent();
	}
	else
	{
		providerName = CDVBString((const char*)&(buffer[4]), service_provider_name_length).getContent();
		serviceName  = CDVBString((const char*)&(buffer[4 + service_provider_name_length + 1]), (2 + buffer[1]) - (4 + service_provider_name_length + 1)).getContent();
	}
	if(serviceName.empty() || serviceName == "."){
		char buf_tmp[16]={0};
		snprintf(buf_tmp,sizeof(buf_tmp),"(0x%04X_0x%04X)",transport_stream_id, service_id);
		serviceName = buf_tmp;
	}


	t_channel_id channel_id;
	int i = 0;
	freq_id_t freq_tmp = freq;
	freq -= 2;
	for(i = 0; i < 6; i++) {
		channel_id = CREATE_CHANNEL_ID64;
		channel = CServiceManager::getInstance()->FindChannel(channel_id);
		if(channel) {
			service_wr = false;
			channel->setName(serviceName);
			channel->setServiceType(real_type);
			channel->scrambled = free_ca;
			break;
		}
		freq++;
	}

	transponder_id_t tpid = CREATE_TRANSPONDER_ID_FROM_SATELLITEPOSITION_ORIGINALNETWORK_TRANSPORTSTREAM_ID( freq, satellitePosition, original_network_id, transport_stream_id);
	if(service_wr) {
		freq = freq_tmp;
		channel_id = CREATE_CHANNEL_ID64;
		DBG("New channel ===== %llx:::%llx %s\n", channel_id, tpid, serviceName.c_str());

		channel = new CZapitChannel (
				serviceName,
				service_id,
				transport_stream_id,
				original_network_id,
				real_type /*service_type*/,
				satellitePosition,
				freq
				);
		CServiceManager::getInstance()->AddChannel(channel);

		channel->scrambled = free_ca;
		//channel = &ret.first->second;
	}

	//FIXME at this point channel never should be NULL
	if(channel == NULL) {
		printf("service_descriptor: BUG ? channel %llx:::%llx %s nor found neither created !\n", channel_id, tpid, serviceName.c_str());
		return;
	}

	//printf("[scan] last tp %llx new %llx\n", last_tpid, tpid);
	if(last_tpid != tpid) {
		last_tpid = tpid;
		tpchange = true;
	}
	if ( providerName.empty() ) {
		unsigned char buff[1024];
		unsigned short network_descriptors_length=0;
		unsigned short pos=0;


		unsigned char filter[DMX_FILTER_SIZE];
		unsigned char mask[DMX_FILTER_SIZE];

		memset(filter, 0x00, DMX_FILTER_SIZE);
		memset(mask, 0x00, DMX_FILTER_SIZE);

		filter[0] = 0x40;
		filter[4] = 0x00;
		mask[0] = 0xFF;
		mask[4] = 0xFF;
		if(tpchange) {
			cDemux * dmx = new cDemux(1);
			dmx->Open(DMX_PSI_CHANNEL);
			if (!((dmx->sectionFilter(0x10, filter, mask, 5) < 0) || (dmx->Read(buff, 1024) < 0))) {
				network_descriptors_length = ((buff[8] & 0x0F) << 8) | buff[9];
				for (pos = 10; pos < network_descriptors_length + 10; pos += buff[pos + 1] + 2) {
					switch (buff[pos]) {
						case 0x40:
							if (buff[pos+1] > 30)
								buff[pos+1] = 30;

							providerName = CDVBString((const char*)&(buff[pos+2]), buff[pos+1]).getContent();
							break;

						default:
							DBG("first_descriptor_tag: %02x\n", buff[pos]);
							break;
					}
				}
			}
			delete dmx;
		} else {
			providerName=lastProviderName;
		}
	}
	
	removeMultipleWhitespaces( providerName );
	// remove space at ende providerName
	if(!providerName.empty()){
		i = 1;
		while (isspace(providerName[providerName.size()-i])){
			i++;
		}
		i--;
		if(i > 0){
			providerName.resize(providerName.size()-i);
		}
	}else{
		const char *unknown_provider_name = "Unknown Provider";
		providerName = CDVBString(unknown_provider_name, strlen(unknown_provider_name)).getContent();
	}

	lastProviderName = providerName;

	//CZapit::getInstance()->SendEvent(CZapitClient::EVT_SCAN_PROVIDER, (void *) lastProviderName.c_str(), lastProviderName.length() + 1);
	CServiceScan::getInstance()->ChannelFound(service_type, lastProviderName, serviceName);

	switch (service_type) {
		case ST_DIGITAL_TELEVISION_SERVICE:
		case ST_DIGITAL_RADIO_SOUND_SERVICE:
		case ST_NVOD_REFERENCE_SERVICE:
		case ST_NVOD_TIME_SHIFTED_SERVICE:
			{
				CZapitBouquet* bouquet;
				int bouquetId;
				char pname[100];
				if (CFrontend::getInstance()->getInfo()->type == FE_QPSK)
					snprintf(pname, 100, "[%c%03d.%d] %s", satellitePosition > 0? 'E' : 'W', abs(satellitePosition)/10, abs(satellitePosition)%10, providerName.c_str());
				else
					snprintf(pname, 100, "%s", providerName.c_str());

				bouquetId = scanBouquetManager->existsBouquet(pname);

				if (bouquetId == -1)
					bouquet = scanBouquetManager->addBouquet(std::string(pname), false);
				else
					bouquet = scanBouquetManager->Bouquets[bouquetId];

#if 0
				lastServiceName = serviceName;
				CZapit::getInstance()->SendEvent(CZapitClient::EVT_SCAN_SERVICENAME, (void *) lastServiceName.c_str(), lastServiceName.length() + 1);
				CZapit::getInstance()->SendEvent(CZapitClient::EVT_SCAN_SERVICENAME, (void *) serviceName.c_str(), serviceName.length() + 1);
#endif
#if 0
				CZapitChannel* chan = scanBouquetManager->findChannelByChannelID(channel_id);
				if(chan)
					bouquet->addService(chan);
#endif
				bouquet->addService(channel);

				break;
			}
		default:
			break;
	}
	if(CZapit::getInstance()->scanPids()) {
		if(tpchange)
			parse_pat();

		channel->resetPids();
		if(!pat_get_pmt_pid(channel)) {
			if(!parse_pmt(channel)) {
				//if(channel->getPreAudioPid() == 0 && channel->getVideoPid() == 0)
				//	printf("[scan] Channel %s dont have A/V pids !\n", channel->getName().c_str());
				if ((channel->getPreAudioPid() != 0) || (channel->getVideoPid() != 0)) {
					channel->setPidsFlag();
				}
			}
		}
	}
	if(service_type == ST_DIGITAL_TELEVISION_SERVICE && !channel->scrambled) {
		CZapit::getInstance()->SetCurrentChannelID(channel->getChannelID());
	}
}

void current_service_descriptor(const unsigned char * const buffer, const t_service_id service_id, const t_transport_stream_id transport_stream_id, const t_original_network_id original_network_id, t_satellite_position satellitePosition, freq_id_t freq, bool free_ca)
{
	bool service_wr = false;
	uint8_t service_type = buffer[2];

	service_type = fix_service_type(service_type); 
	uint8_t real_type = service_type;

#if 0
	switch ( scanType ) {
		case CZapitClient::ST_TVRADIO:
			if ( (service_type == 1 ) || (service_type == 2) )
				service_wr=true;
			break;
		case CZapitClient::ST_TV:
			if ( service_type == 1 )
				service_wr=true;
			break;
		case CZapitClient::ST_RADIO:
			if ( service_type == 2 )
				service_wr=true;
			break;
		case CZapitClient::ST_ALL:
			service_wr=true;
			break;
	}
#endif
	if ( (service_type == 1 ) || (service_type == 2) )
		service_wr=true;

	if ( !service_wr )
		return;

	if(CServiceManager::getInstance()->FindCurrentChannel(CREATE_CHANNEL_ID64))
		return;

	uint8_t service_provider_name_length = buffer[3];

	std::string providerName((const char*)&(buffer[4]), service_provider_name_length);
	std::string serviceName;
//	std::string satelliteName = "unknown";

	bool in_blacklist = false;

	if (check_blacklisted(providerName)) {
		in_blacklist = true;
	}else if((check_blacklisted_digital_plus(original_network_id, transport_stream_id))){
		providerName = "Digital+";
		in_blacklist = true;
	}

	if (in_blacklist) {
		if (((unsigned char)buffer[4 + service_provider_name_length + 1]) >= 0x20) // no encoding info
			serviceName  = CDVBString(("\x05" + std::string((const char*)&(buffer[4 + service_provider_name_length + 1]), (2 + buffer[1]) - (4 + service_provider_name_length + 1))).c_str(), (2 + buffer[1]) - (4 + service_provider_name_length + 1) + 1).getContent(); // add artificial encoding info
		else
			serviceName  = CDVBString((const char*)&(buffer[4 + service_provider_name_length + 1]), (2 + buffer[1]) - (4 + service_provider_name_length + 1)).getContent();
	} else {
		serviceName  = CDVBString((const char*)&(buffer[4 + service_provider_name_length + 1]), (2 + buffer[1]) - (4 + service_provider_name_length + 1)).getContent();
	}
	if(serviceName.empty() || serviceName == "." ){
		char buf_tmp[16]={0};
		snprintf(buf_tmp,sizeof(buf_tmp),"(0x%04X_0x%04X)",transport_stream_id, service_id);
		serviceName = buf_tmp;
	}

	CZapitChannel *	channel = new CZapitChannel(serviceName,
			service_id,
			transport_stream_id,
			original_network_id,
			real_type /*service_type*/,
			satellitePosition,
			freq
			);
	CServiceManager::getInstance()->AddCurrentChannel(channel);
	channel->scrambled = free_ca;
}

/* 0x49 */
void country_availability_descriptor(const unsigned char * const)
{
}

/* 0x4A */
void linkage_descriptor(const unsigned char * const)
{
}

/* 0x4B */
int NVOD_reference_descriptor(
	const unsigned char * const buffer,
	const unsigned int num,
	t_transport_stream_id * const tsid,
	t_original_network_id * const onid,
	t_service_id * const sid)
{
	if ((unsigned int)(buffer[1] / 6) + 1 >= num) {
		*tsid = (buffer[2 + (6 * num)] << 16) | buffer[3 + (6 * num)];
		*onid = (buffer[4 + (6 * num)] << 16) | buffer[5 + (6 * num)];
		*sid =  (buffer[6 + (6 * num)] << 16) | buffer[7 + (6 * num)];
		return 0;
	}

	return -1;
}

/* 0x4C */
void time_shifted_service_descriptor(const unsigned char * const)
{
}

/* 0x4D */
void short_event_descriptor(const unsigned char * const)
{
}

/* 0x4E */
void extended_event_descriptor(const unsigned char * const)
{
}

/* 0x4F */
void time_shifted_event_descriptor(const unsigned char * const)
{
}

/* 0x50 */
void component_descriptor(const unsigned char * const)
{
}

/* 0x51 */
void mosaic_descriptor(const unsigned char * const)
{
}

/* 0x52 */
void stream_identifier_descriptor(const unsigned char * const)
{
}

/* 0x53 */
void CA_identifier_descriptor(const unsigned char * const)
{
}

/* 0x54 */
void content_descriptor(const unsigned char * const)
{
}

/* 0x55 */
void parental_rating_descriptor(const unsigned char * const)
{
}

/* 0x56 */
void teletext_descriptor(const unsigned char * const)
{
}

/* 0x57 */
void telephone_descriptor(const unsigned char * const)
{
}

/* 0x58 */
void local_time_offset_descriptor(const unsigned char * const)
{
}

/* 0x59 */
void subtitling_descriptor(const unsigned char * const)
{
}

/* 0x5A */
int terrestrial_delivery_system_descriptor(const unsigned char * const)
{
	if (CFrontend::getInstance()->getInfo()->type != FE_OFDM)
		return -1;

	/* TODO */

	return 0;
}

/* 0x5B */
void multilingual_network_name_descriptor(const unsigned char * const)
{
}

/* 0x5C */
void multilingual_bouquet_name_descriptor(const unsigned char * const)
{
}

/* 0x5D */
void multilingual_service_name_descriptor(const unsigned char * const)
{
}

/* 0x5E */
void multilingual_component_descriptor(const unsigned char * const)
{
}

/* 0x5F */
void private_data_specifier_descriptor(const unsigned char * const)
{
}

/* 0x60 */
void service_move_descriptor(const unsigned char * const)
{
}

/* 0x61 */
void short_smoothing_buffer_descriptor(const unsigned char * const)
{
}

/* 0x62 */
void frequency_list_descriptor(const unsigned char * const)
{
}

/* 0x63 */
void partial_transport_stream_descriptor(const unsigned char * const)
{
}

/* 0x64 */
void data_broadcast_descriptor(const unsigned char * const)
{
}

/* 0x65 */
void CA_system_descriptor(const unsigned char * const)
{
}

/* 0x66 */
void data_broadcast_id_descriptor(const unsigned char * const)
{
}

/* 0x67 */
void transport_stream_descriptor(const unsigned char * const)
{
}

/* 0x68 */
void DSNG_descriptor(const unsigned char * const)
{
}

/* 0x69 */
void PDC_descriptor(const unsigned char * const)
{
}

/* 0x6A */
void AC3_descriptor(const unsigned char * const)
{
}

/* 0x6B */
void ancillary_data_descriptor(const unsigned char * const)
{
}

/* 0x6C */
void cell_list_descriptor(const unsigned char * const)
{
}

/* 0x6D */
void cell_frequency_link_descriptor(const unsigned char * const)
{
}

/* 0x6E */
void announcement_support_descriptor(const unsigned char * const)
{
}

/* 0x6F ... 0x7F: reserved */
/* 0x80 ... 0xFE: user private */
/* 0xFF: forbidden */
