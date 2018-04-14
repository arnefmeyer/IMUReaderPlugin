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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "IMUReader.h"

class IMUReaderEditor;


class IMUFrame
{
public:
	IMUFrame(juce::int64 idx, juce::int64 ts, int experiment_, int recording_, float ax_, float ay_, float az_, float gx_, float gy_, float gz_, float mx_, float my_, float mz_)
	{
		index = idx;
		timestamp = ts;
		experiment = experiment_;
		recording = recording_;
		ax = ax_;
		ay = ay_;
		az = az_;
		gx = gx_;
		gy = gy_;
		gz = gz_;
		mx = mx_;
		my = my_;
		mz = mz_;
	}

	String asCsv(bool newline)
	{
		String line = String::formatted("%lld,%lld,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f", index, timestamp, experiment, recording, ax, ay, az, gx, gy, gz, mx, my, mz);

		if (newline)
			 line += String("\n");

		return line;
	}

private:
	juce::int64 index;
	juce::int64 timestamp;
	int experiment;
	int recording;
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float mx;
	float my;
	float mz;
};


class DiskThread : public Thread
{
public:
	DiskThread(File dest) : Thread("DiskThread")
	{
		destPath = dest;
		timestampFile = File();
		frameBuffer.clear();
	}

	~DiskThread()
	{
		stopThread(1000);
		frameBuffer.clear();
	}

	void setDestinationPath(File &f)
	{
		lock.enter();
		destPath = File(f);
		lock.exit();
	}

	void createTimestampFile(String name = "IMU_data")
	{
		char filePath[1024];
		sprintf(filePath, "%s/%s.csv", destPath.getFullPathName().toRawUTF8(), name.toRawUTF8());
		timestampFile = File(filePath);

		if (!timestampFile.exists())
		{
			timestampFile.create();
			timestampFile.appendText("# index, timestamp, experiment, recording, ax, ay, az, gx, gy, gz, mx, my, mz\n");
		}
	}

	void run()
	{
		IMUFrame *frame;

		while (!threadShouldExit())
		{
			frame = NULL;

			lock.enter();
			if (frameBuffer.size() > 0)
			{
				frame = frameBuffer.removeAndReturn(0);
			}
			lock.exit();

			if (frame != NULL)
			{
				if (timestampFile != File::nonexistent)
				{
					timestampFile.appendText(frame->asCsv(true));
				}

				delete frame;
			}
		}
	}

	void addFrame(IMUFrame *frame)
	{
		lock.enter();
		frameBuffer.add(frame);
		lock.exit();
	}

private:
	OwnedArray<IMUFrame> frameBuffer;
	File destPath;
	File timestampFile;
	CriticalSection lock;
	bool threadRunning;
};


IMUReader::IMUReader()
    : GenericProcessor("IMU Reader"), Thread("IMUReaderThread"),
	  isRecording(false), device("/dev/ttyACM0"), deviceConnected(false), frameCounter(0)

{
    setProcessorType(PROCESSOR_TYPE_SOURCE);

	File recPath = CoreServices::RecordNode::getRecordingPath();
	diskThread = new DiskThread(File(recPath.getFullPathName()));

	startThread();
}


IMUReader::~IMUReader()
{
	if (isThreadRunning())
	{
		signalThreadShouldExit();
		stopThread(1000);
	}
	serial.close();

	if (diskThread != NULL)
	{
		delete diskThread;
	}
}


AudioProcessorEditor* IMUReader::createEditor()
{
    editor = new IMUReaderEditor(this, true);

    return editor;
}


void IMUReader::updateSettings()
{
	if (editor != NULL)
	{
		editor->update();
	}
}


bool IMUReader::enable()
{
	if (!changeStatus(RUNNING))
		std::cout << ">>> IMUReader::enable could not change status\n";
}


bool IMUReader::disable()
{
	if (!changeStatus(WAITING))
		std::cout << ">>> IMUReader::disable could not change status\n";
}


void IMUReader::startRecording()
{
	if (deviceConnected)
	{
		File recPath = CoreServices::RecordNode::getRecordingPath();

		File frameFile = File(recPath.getFullPathName());
		if (!frameFile.isDirectory())
		{
			Result result = frameFile.createDirectory();
			if (result.failed())
			{
				std::cout << ">>> IMUReader: failed to create recording path!" << "\n";
			}
		}

		lock.enter();

		diskThread->setDestinationPath(frameFile);
		diskThread->createTimestampFile();
		diskThread->startThread();

		isRecording = true;

		lock.exit();

		if (!changeStatus(RECORDING))
			std::cout << ">>> IMUReader::startRecording could not change status\n";
	}

	IMUReaderEditor* e = (IMUReaderEditor*) editor.get();
	e->disableControls();
}


void IMUReader::stopRecording()
{
	if (deviceConnected)
	{
		if (!changeStatus(RUNNING))
			std::cout << ">>> IMUReader::stopRecording could not change status\n";

		lock.enter();
		isRecording = false;
		lock.exit();
	}

	IMUReaderEditor* e = (IMUReaderEditor*) editor.get();
	e->enableControls();
}


void IMUReader::process(AudioSampleBuffer& buffer)
{
}


void IMUReader::startIMU()
{
	changeStatus(RUNNING);
}


void IMUReader::stopIMU()
{
	changeStatus(WAITING);
}


bool IMUReader::isConnected()
{
	return deviceConnected;
}


String IMUReader::getDevice()
{
	return device;
}


int IMUReader::setDevice(String dev)
{
	int status;

	if (dev == "" || dev == "None")
	{
		serial.close();
		device = "";
		status = 0;
		deviceConnected = false;
	}
	else if (dev != getDevice() || !deviceConnected)
	{
		device = dev;

		if (deviceConnected)
		{
			serial.close();
		}

		if (serial.setup(device.toStdString(), 115200))
		{
			deviceConnected = true;
			status = 0;
		}
		else
		{
			deviceConnected = false;
			status = 1;
		}
	}

	return status;
}


void IMUReader::setDirectoryName(String name)
{
	if (name != getDirectoryName())
	{
		if (File::createLegalFileName(name) == name)
		{
			dirName = name;
		}
		else
		{
			std::cout << "IMUReader: invalid directory name: " << name.toStdString() << "\n";
		}
	}
}


String IMUReader::getDirectoryName()
{
	return dirName;
}


void IMUReader::run()
{
	unsigned char buf[1024];
	int n;
	int status;
	bool add = false;

	std::stringstream ss;

    while (true)
    {
		if (deviceConnected)
		{
			memset(buf, ' ', 1024);
			n = serial.readBytes(&buf[0], 1024);

			if ((n != OF_SERIAL_ERROR) && (n > 0))
			{
				for (int i=0; i<1024; i++)
				{
					if (int(buf[i]) != 32) // skip spaces
					{
						if (int(buf[i]) == 62) // ">"
						{
							// beginning of imu frame data
							add = true;
						}
						else if (int(buf[i]) == 60) // "<"
						{	
							// ending of imu frame -> create frame object
							StringArray tokens;
							tokens.addTokens(ss.str().c_str(), ",", "");
							status = tokens[0].getIntValue();

							if (status == RECORDING and isRecording and diskThread->isThreadRunning())
							{
								IMUFrame* frame = new IMUFrame(tokens[1].getLargeIntValue(),
														       tokens[2].getLargeIntValue(),
															   CoreServices::RecordNode::getExperimentNumber(),
															   CoreServices::RecordNode::getRecordingNumber(),
															   tokens[3].getFloatValue(),
															   tokens[4].getFloatValue(),
															   tokens[5].getFloatValue(),
															   tokens[6].getFloatValue(),
															   tokens[7].getFloatValue(),
															   tokens[8].getFloatValue(),
															   tokens[9].getFloatValue(),
															   tokens[10].getFloatValue(),
															   tokens[11].getFloatValue());
								diskThread->addFrame(frame);
							}

							frameCounter += 1;

							ss.str("");
							ss.clear();
							add = false;
						}
						else if (add)
						{
							ss << buf[i];
						}

					}
				}
			}
		}

		if (threadShouldExit())
		{
			changeStatus(WAITING);
			break;
		}
    }
}


juce::int64 IMUReader::getFrameCount()
{
	return frameCounter;
}


void IMUReader::saveCustomParametersToXml(XmlElement* xml)
{
    xml->setAttribute("Type", "IMUReader");

    XmlElement* paramXml = xml->createNewChildElement("PARAMETERS");
    paramXml->setAttribute("Device", device);
}


void IMUReader::loadCustomParametersFromXml()
{
	forEachXmlChildElementWithTagName(*parametersAsXml,	paramXml, "PARAMETERS")
	{
		if (paramXml->hasAttribute("Device"))
			setDevice(paramXml->getStringAttribute("Device"));
	}

	updateSettings();
}


bool IMUReader::changeStatus(status_t status)
{
	int n;
	bool success = true;

	if (deviceConnected)
	{
		// see arduino/teensy sketches for status definition
		unsigned char cmd_waiting[] = "1x";
		unsigned char cmd_running[] = "2x";
		unsigned char cmd_recording[] = "3x";

		switch (status)
		{
			case WAITING:
				n = serial.writeBytes(&cmd_waiting[0], 2);
				break;
			case RUNNING:
				n = serial.writeBytes(&cmd_running[0], 2);
				frameCounter = 0;
				break;
			case RECORDING:
				n = serial.writeBytes(&cmd_recording[0], 2);
				frameCounter = 0;
				break;
			default:
				;
		}

		currentStatus = status;
	}
	else
	{
		success = false;
	}

	return success;
}


