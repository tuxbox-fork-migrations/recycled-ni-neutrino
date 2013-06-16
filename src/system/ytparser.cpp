/*
        Copyright (C) 2013 CoolStream International Ltd

        License: GPLv2

        This program is free software; you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation;

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program; if not, write to the Free Software
        Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <set>
#include <map>
#include <vector>
#include <bitset>
#include <string>

#include "ytparser.h"

#if LIBCURL_VERSION_NUM < 0x071507
#include <curl/types.h>
#endif

#define URL_TIMEOUT 60

std::string cYTVideoUrl::GetUrl()
{
	std::string fullurl = url;
	fullurl += "&signature=";
	fullurl += sig;
	return fullurl;
}

void cYTVideoInfo::Dump()
{
	printf("id: %s\n", id.c_str());
	printf("author: %s\n", author.c_str());
	printf("title: %s\n", title.c_str());
	printf("duration: %d\n", duration);
	//printf("description: %s\n", description.c_str());
	printf("urls: %d\n", formats.size());
	for (yt_urlmap_iterator_t it = formats.begin(); it != formats.end(); ++it) {
		printf("format %d type [%s] url %s\n", it->first, it->second.type.c_str(), it->second.GetUrl().c_str());
	}
	printf("===================================================================\n");
}

std::string cYTVideoInfo::GetUrl(int fmt)
{
	yt_urlmap_iterator_t it;
	if (fmt) {
		if ((it = formats.find(fmt)) != formats.end())
			return it->second.GetUrl();
		return "";
	}
	if ((it = formats.find(37)) != formats.end())
		return it->second.GetUrl();
	if ((it = formats.find(22)) != formats.end())
		return it->second.GetUrl();
	if ((it = formats.find(18)) != formats.end())
		return it->second.GetUrl();
	return "";
}

cYTFeedParser::cYTFeedParser()
{
	thumbnail_dir = "/tmp/ytparser";
	parsed = false;
	feedmode = -1;
	tquality = "mqdefault";
	max_results = 25;
}

cYTFeedParser::~cYTFeedParser()
{
}

size_t cYTFeedParser::CurlWriteToString(void *ptr, size_t size, size_t nmemb, void *data)
{
        std::string* pStr = (std::string*) data;
        pStr->append((char*) ptr, nmemb);
        return size*nmemb;
}

bool cYTFeedParser::getUrl(std::string &url, std::string &answer)
{
	CURL * curl_handle = curl_easy_init();
	curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, &cYTFeedParser::CurlWriteToString);
	curl_easy_setopt(curl_handle, CURLOPT_FILE, (void *)&answer);
	curl_easy_setopt(curl_handle, CURLOPT_FAILONERROR, 1);
	curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, URL_TIMEOUT);
	curl_easy_setopt(curl_handle, CURLOPT_NOSIGNAL, (long)1);

	char cerror[CURL_ERROR_SIZE];
	curl_easy_setopt(curl_handle, CURLOPT_ERRORBUFFER, cerror);

	printf("try to get [%s] ...\n", url.c_str());
	CURLcode httpres = curl_easy_perform(curl_handle);

	curl_easy_cleanup(curl_handle);

	printf("http: res %d size %d\n", httpres, answer.size());

	if (httpres != 0 || answer.empty()) {
		printf("error: %s\n", cerror);
		return false;
	}
	return true;
}

bool cYTFeedParser::DownloadUrl(std::string &url, std::string &file)
{
	FILE * fp = fopen(file.c_str(), "wb");
	if (fp == NULL) {
		perror(file.c_str());
		return false;
	}
	CURL * curl_handle = curl_easy_init();
	curl_easy_setopt(curl_handle, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl_handle, CURLOPT_FILE, fp);
	curl_easy_setopt(curl_handle, CURLOPT_FAILONERROR, 1);
	curl_easy_setopt(curl_handle, CURLOPT_TIMEOUT, URL_TIMEOUT);
	curl_easy_setopt(curl_handle, CURLOPT_NOSIGNAL, (long)1);

	char cerror[CURL_ERROR_SIZE];
	curl_easy_setopt(curl_handle, CURLOPT_ERRORBUFFER, cerror);

	printf("try to get [%s] ...\n", url.c_str());
	CURLcode httpres = curl_easy_perform(curl_handle);

	double dsize;
	curl_easy_getinfo(curl_handle, CURLINFO_SIZE_DOWNLOAD, &dsize);
	curl_easy_cleanup(curl_handle);
	fclose(fp);

	printf("http: res %d size %f.\n", httpres, dsize);

	if (httpres != 0) {
		printf("curl error: %s\n", cerror);
		unlink(file.c_str());
		return false;
	}
	return true;
}

void cYTFeedParser::decodeUrl(std::string &url)
{
	CURL * curl_handle = curl_easy_init();
	char * str = curl_easy_unescape(curl_handle, url.c_str(), 0, NULL);
	curl_easy_cleanup(curl_handle);
	if(str)
		url = str;
	curl_free(str);
}

void cYTFeedParser::encodeUrl(std::string &txt)
{
	CURL * curl_handle = curl_easy_init();
	char * str = curl_easy_escape(curl_handle, txt.c_str(), txt.length());
	curl_easy_cleanup(curl_handle);
	if(str)
		txt = str;
	curl_free(str);
}

void cYTFeedParser::splitString(std::string &str, std::string delim, std::vector<std::string> &strlist, int start)
{
	strlist.clear();
	unsigned int end = 0;
	while ((end = str.find(delim, start)) != std::string::npos) {
		strlist.push_back(str.substr(start, end - start));
		start = end + delim.size();
	}
	strlist.push_back(str.substr(start));
}

void cYTFeedParser::splitString(std::string &str, std::string delim, std::map<std::string,std::string> &strmap, int start)
{
	unsigned int end = 0;
	if ((end = str.find(delim, start)) != std::string::npos) {
		strmap[str.substr(start, end - start)] = str.substr(end - start + delim.size());
	}
}

bool cYTFeedParser::saveToFile(const char * name, std::string str)
{
	FILE * fp = fopen(name, "w+");
	if (fp) {
		fprintf(fp, "%s", str.c_str());
		fclose(fp);
		return false;
	}
	printf("cYTFeedParser::saveToFile: failed to open %s\n", name);
	return false;
}

std::string cYTFeedParser::getXmlName(xmlNodePtr node)
{
	std::string result;
	char * name = xmlGetName(node);
	if (name)
		result = name;
	return result;
}

std::string cYTFeedParser::getXmlAttr(xmlNodePtr node, const char * attr)
{
	std::string result;
	char * value = xmlGetAttribute(node, attr);
	if (value)
		result = value;
	return result;
}

std::string cYTFeedParser::getXmlData(xmlNodePtr node)
{
	std::string result;
	char * value = xmlGetData(node);
	if (value)
		result = value;
	return result;
}

bool cYTFeedParser::parseFeedXml(std::string &answer)
{
	xmlDocPtr answer_parser = parseXmlFile(curfeedfile.c_str());
	if (answer_parser == NULL)
		answer_parser = parseXml(answer.c_str());

	if (answer_parser == NULL) {
		printf("failed to parse xml\n");
		return false;
	}
	next.clear();
	prev.clear();
	total.clear();
	start.clear();
	xmlNodePtr entry = xmlDocGetRootElement(answer_parser)->xmlChildrenNode;
	while (entry) {
		std::string name = getXmlName(entry);
#ifdef DEBUG_PARSER
		printf("entry: %s\n", name.c_str());
#endif
		if (name == "openSearch:startIndex") {
			start = getXmlData(entry);
			printf("start %s\n", start.c_str());
		} else if (name == "openSearch:totalResults") {
			total = getXmlData(entry);
			printf("total %s\n", total.c_str());
		}
		else if (name == "link") {
			std::string link = getXmlAttr(entry, "rel");
			if (link == "next") {
				next = getXmlAttr(entry, "href");
				printf("	next [%s]\n", next.c_str());
			} else if (link == "previous") {
				prev = getXmlAttr(entry, "href");
				printf("	prev [%s]\n", prev.c_str());
			}
		}
		else if (name != "entry") {
			entry = entry->xmlNextNode;
			continue;
		}
		xmlNodePtr node = entry->xmlChildrenNode;
		cYTVideoInfo vinfo;
		std::string thumbnail;
		while (node) {
			name = getXmlName(node);
#ifdef DEBUG_PARSER
			printf("	node: %s\n", name.c_str());
#endif
			if (name == "title") {
#ifdef DEBUG_PARSER
				printf("		title [%s]\n", getXmlData(node).c_str());
#endif
				vinfo.title = getXmlData(node);
			}
			else if (name == "published") {
				vinfo.published = getXmlData(node).substr(0, 10);
			}
			else if (name == "author") {
				xmlNodePtr author = node->xmlChildrenNode;
				while(author) {
					name = getXmlName(author);
					if (name == "name") {
#ifdef DEBUG_PARSER
						printf("		author [%s]\n", getXmlData(author).c_str());
#endif
						vinfo.author = getXmlData(author);
					}
					author = author->xmlNextNode;
				}
			}
			else if (name == "media:group") {
				xmlNodePtr media = node->xmlChildrenNode;
				while (media) {
					name = getXmlName(media);
					if (name == "media:description") {
						vinfo.description = getXmlData(media).c_str();
					}
					else if (name == "media:category") {
						if (vinfo.category.size() < 3)
							vinfo.category = getXmlData(media).c_str();
					}
					else if (name == "yt:videoid") {
#ifdef DEBUG_PARSER
						printf("		id [%s]\n", getXmlData(media).c_str());
#endif
						vinfo.id = getXmlData(media).c_str();
					}
					else if (name == "media:thumbnail") {
						/* save first found */
						if (thumbnail.empty())
							thumbnail = getXmlAttr(media, "url");

						/* check wanted quality */
						if (tquality == getXmlAttr(media, "yt:name")) {
							vinfo.thumbnail = getXmlAttr(media, "url");
#ifdef DEBUG_PARSER
							printf("vinfo.thumbnail [%s]\n", vinfo.thumbnail.c_str());
#endif
						}
					}
					else if (name == "yt:duration") {
						vinfo.duration = atoi(getXmlAttr(media, "seconds").c_str());
					}
#if 0
					else if (name == "media:player") {
						std::string url = getXmlAttr(media, "url");
						printf("		media:player [%s]\n", url.c_str());
					}
					else if (name == "media:title") {
					}
#endif
					media = media->xmlNextNode;
				}
			}
			node = node->xmlNextNode;
		}
		if (!vinfo.id.empty()) {
			/* save first one, if wanted not found */
			if (vinfo.thumbnail.empty())
				vinfo.thumbnail = thumbnail;
			if (ParseVideoInfo(vinfo))
				videos.push_back(vinfo);
		}
		entry = entry->xmlNextNode;
	}
	xmlFreeDoc(answer_parser);
	parsed = !videos.empty();
	return parsed;
}

bool cYTFeedParser::supportedFormat(int fmt)
{
	if((fmt == 37) || (fmt == 22) || (fmt == 18))
		return true;
	return false;
}

bool cYTFeedParser::decodeVideoInfo(std::string &answer, cYTVideoInfo &vinfo)
{
	bool ret = false;
	decodeUrl(answer);
#if 0
	std::string infofile = thumbnail_dir;
	infofile += "/";
	infofile += vinfo.id;
	infofile += ".txt"
	saveToFile(infofile.c_str(), answer);
#endif
	if(answer.find("token=") == std::string::npos)
		return ret;

	//FIXME check expire
	std::vector<std::string> ulist;
	unsigned fmt = answer.find("url_encoded_fmt_stream_map=");
	if (fmt != std::string::npos) {
		fmt = answer.find("=", fmt);
		splitString(answer, ",", ulist, fmt+1);
		for (unsigned i = 0; i < ulist.size(); i++) {
#if 0 // to decode all params
			decodeUrl(ulist[i]);
			printf("URL: %s\n", ulist[i].c_str());
#endif
			std::map<std::string,std::string> smap;
			std::vector<std::string> uparams;
			splitString(ulist[i], "&", uparams);
			if (uparams.size() < 3)
				continue;
			for (unsigned j = 0; j < uparams.size(); j++) {
				decodeUrl(uparams[j]);
#ifdef DEBUG_PARSER
				printf("	param: %s\n", uparams[j].c_str());
#endif
				splitString(uparams[j], "=", smap);
			}
#ifdef DEBUG_PARSER
			printf("=========================================================\n");
#endif
			cYTVideoUrl yurl;
			yurl.url = smap["url"];
			yurl.sig = smap["sig"];
			int id = atoi(smap["itag"].c_str());
			if (supportedFormat(id) && !yurl.url.empty() && !yurl.sig.empty()) {
				yurl.quality = smap["quality"];
				yurl.type = smap["type"];
				vinfo.formats.insert(yt_urlmap_pair_t(id, yurl));
				ret = true;
			}
		}
	}
	return ret;
}

bool cYTFeedParser::ParseFeed(std::string &url)
{
	videos.clear();

	std::string answer;
	curfeedfile = thumbnail_dir;
	curfeedfile += "/";
	curfeedfile += curfeed;
	curfeedfile += ".xml";
#ifdef CACHE_FILES
	if(!DownloadUrl(url, cfile))
		return false;
#else
	if (!getUrl(url, answer))
		return false;
#endif
	return parseFeedXml(answer);
}

bool cYTFeedParser::ParseFeed(yt_feed_mode_t mode, std::string search, std::string vid)
{
	std::string url = "http://gdata.youtube.com/feeds/api/standardfeeds/";
	bool append_res = true;
	if (mode < FEED_LAST) {
		switch(mode) {
			case TOP_RATED:
				curfeed = "top_rated";
				break;
			case TOP_FAVORITES:
				curfeed = "top_favorites";
				break;
			case MOST_SHARED:
				curfeed = "most_shared";
				break;
			case MOST_POPULAR:
			default:
				curfeed = "most_popular";
				break;
			case MOST_RESENT:
				curfeed = "most_recent";
				break;
			case MOST_DISCUSSED:
				curfeed = "most_discussed";
				break;
			case MOST_RESPONDED:
				curfeed = "most_responded";
				break;
			case RECENTLY_FEATURED:
				curfeed = "recently_featured";
				break;
			case ON_THE_WEB:
				curfeed = "on_the_web";
				break;
		}
		if (!region.empty()) {
			url += region;
			url += "/";
		}
		url += curfeed;
		url += "?";
	}
	else if (mode == NEXT) {
		if (next.empty())
			return false;
		url = next;
		append_res = false;
	}
	else if (mode == PREV) {
		if (prev.empty())
			return false;
		url = prev;
		append_res = false;
	}
	else if (mode == RELATED) {
		if (vid.empty())
			return false;
		url = "http://gdata.youtube.com/feeds/api/videos/";
		url += vid;
		url += "/related?";
	}
	else if (mode == SEARCH) {
		if (search.empty())
			return false;
		encodeUrl(search);
		url = "http://gdata.youtube.com/feeds/api/videos?q=";
		url += search;
		url += "&";
	}

	feedmode = mode;
	if (append_res) {
		url += "v=2&max-results=";
		char res[10];
		sprintf(res, "%d", max_results);
		url+= res;
	}

	return ParseFeed(url);
}

bool cYTFeedParser::ParseVideoInfo(cYTVideoInfo &vinfo)
{
	bool ret = false;
	std::vector<std::string> estr;
	estr.push_back("&el=embedded");
	estr.push_back("&el=vevo");
	estr.push_back("&el=detailpage");

	for (unsigned i = 0; i < estr.size(); i++) {
		std::string vurl = "http://www.youtube.com/get_video_info?video_id=";
		vurl += vinfo.id;
		vurl += estr[i];
		vurl += "&ps=default&eurl=&gl=US&hl=en";
		printf("cYTFeedParser::ParseVideoInfo: get [%s]\n", vurl.c_str());
		std::string answer;
		if (!getUrl(vurl, answer))
			continue;
		ret = decodeVideoInfo(answer, vinfo);
		if (ret)
			break;
	}
	return ret;
}

bool cYTFeedParser::DownloadThumbnails()
{
	bool ret = false;
	if (mkdir(thumbnail_dir.c_str(), 0755)) {
		perror(thumbnail_dir.c_str());
		//return ret;
	}
	for (unsigned i = 0; i < videos.size(); i++) {
		if (!videos[i].thumbnail.empty()) {
			std::string fname = thumbnail_dir;
			fname += "/";
			fname += videos[i].id;
			fname += ".jpg";
			bool found = !access(fname.c_str(), F_OK);
			if (!found)
				found = DownloadUrl(videos[i].thumbnail, fname);
			if (found)
				videos[i].tfile = fname;
			ret |= found;
		}
	}
	return ret;
}

bool cYTFeedParser::GetVideoUrls()
{
	bool ret = false;
	for (unsigned i = 0; i < videos.size(); i++) {
		ret |= ParseVideoInfo(videos[i]);
	}
	return ret;
}

void cYTFeedParser::Cleanup(bool delete_thumbnails)
{
	printf("cYTFeedParser::Cleanup: %d videos\n", videos.size());
	if (delete_thumbnails) {
		for (unsigned i = 0; i < videos.size(); i++) {
			unlink(videos[i].tfile.c_str());
		}
	}
	unlink(curfeedfile.c_str());
	videos.clear();
	parsed = false;
	feedmode = -1;
}

void cYTFeedParser::Dump()
{
	printf("feed: %d videos\n", videos.size());
	for (unsigned i = 0; i < videos.size(); i++)
		videos[i].Dump();
}
