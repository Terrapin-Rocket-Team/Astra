//
// Created by aditya on 10/30/25.
//
#include "FileHandler.h"

namespace astra
{
	FileHandler::FileHandler(const char *path, StorageMedium medium, uint8_t csPin, bool prefix)
		: _path(path), _medium(medium), _csPin(csPin), _prefix(prefix) {}
	FileHandler::~FileHandler(){
		end();
	}

	bool FileHandler::begin(){
		if (_ready){
			return true;
		}
		bool success = false;

		switch (_medium){
			case StorageMedium::SD:
				success = initSD();
				break;
			case StorageMedium::EMMC:
				success = initEMMC();
				break;
			case StorageMedium::Flash:
				success = initFlash();
				break;
		}

		if(success){
			success = openFile();
			if(!success){
				Serial.println("Failed to open file: ");
				Serial.println(_path);
			}
		else{
			Serial.println("Failed to initialize storage medium on pin: ");
			Serial.println(_csPin);
		}
		_ready = success;
		return _ready;
	}

	bool FileHandler::end(){
		if(_file){
			_file.flush();
			_file.close();
		}
		_ready = false;
		return true;
	}

	bool FileHandler::ok() const{
		return _ready && _file;
	}

	bool FileHandler::wantsPrefix() const{
		return _prefix;
	}

	size_t FileHandler::write(uint8_t b){
		if(!_ready || !_file){
			return 0;
		}
		return _file.write(b);
	}

	void FileHandler::flush(){
		if(_file){
			_file.flush();
		}
	}

	bool FileHandler::initSD(){
		SdSpiConfig config(_csPin, SHARED_SPI, SPI_CLOCK);
		if(!_sd.begin(config)){
			Serial.println("Failed to initialize SD card");
			Serial.print("CS Pin: ");
			Serial.println(_csPin);

			if(_sd.card()->errorCode()){
				Serial.print("SD error: 0x");
				Serial.println(_sd.card()->errorCode(), HEX);
				Serial.print("error data: 0x");
				Serial.println(_sd.card()->errorData(), HEX);
			}
			return false;
		}
		Serial.println("SD initialized");
		return true;
	}

	bool FileHandler::initEMMC(){}

	bool FileHandler::initFlash(){}

	bool FileHandler::openFile(){
		if (_file) {
			_file.close();
		}

		_file = _sd.open(_path, FILE_WRITE);

		if (!_file) {
			Serial.print("Failed to open file: ");
			Serial.println(_path);
			return false;
		}

		Serial.print("File opened successfully: ");
		Serial.println(_path);

		if (_file.size() == 0) {
			Serial.println("New file created");
		}
		else {
			Serial.print("Appending to existing file (");
			Serial.print(_file.size());
			Serial.println(" bytes)");
		}

		return true;
	}

}
