/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2014 Open Ephys

    ------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef IMUREADER_H_INCLUDED
#define IMUREADER_H_INCLUDED

#ifdef _WIN32
#include <Windows.h>
#endif

#include <ProcessorHeaders.h>
#include "IMUReaderEditor.h"
#include <SerialLib.h>

class DiskThread;


enum status_t {WAITING=1, RUNNING=2, RECORDING=3};


class IMUReader : public GenericProcessor,  public Thread
{
public:

    IMUReader();
    ~IMUReader();

    bool isSource()
    {
        return true;
    }

    bool isSink()
    {
        return false;
    }

    void process(AudioSampleBuffer& buffer);

	bool enable();
	bool disable();

	void startRecording();
	void stopRecording();

	void run();

	AudioProcessorEditor* createEditor();
	void updateSettings();

	void startIMU();
	void stopIMU();
	bool isConnected();

	juce::int64 getFrameCount();

	String getDevice();
	int setDevice(String path);

	void setDirectoryName(String name);
	String getDirectoryName();

	void saveCustomParametersToXml(XmlElement* parentElement);
	void loadCustomParametersFromXml();

private:
	juce::String device;
	bool deviceConnected;
	bool threadRunning;
	bool isRecording;
	String dirName;
	int currentFormatIndex;
	DiskThread *diskThread;
	status_t currentStatus;
	juce::int64 frameCounter;

	CriticalSection lock;

	bool changeStatus(status_t s);
	ofSerial serial;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(IMUReader);
};


#endif  // IMUREADER_H_INCLUDED

