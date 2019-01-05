# OpenDVCbasedDAC
Distributed Video Coding Using Distributed Arithmetic Coding

in config.h:

Hall Monitor：
 ISCHANGE				0					
 PSOTION				2					
 OVERLAP				0.08				
 HIGHMOTACT				0

Coast Guard：
 ISCHANGE				0
 PSOTION				2
 OVERLAP				0.10
 HIGHMOTACT				0

Foreman：
 ISCHANGE				1
 PSOTION				2
 OVERLAP				0.025
 HIGHMOTACT				0

Soccer：
 ISCHANGE				0
 PSOTION				2
 OVERLAP				0.02
 HIGHMOTACT				1

File information introduction：

xxx_qcif.yuv				test video sequence

rec.yuv					decoded H.264 video

test.264				H.264 relevant file

wz.bin					WZ frames binary file, Store WZ binary data at fixed length

leakybucketparam.cfg			H.264 relevant file

crc.dat					CRC parity bit

log.dat					H.264 relevant file

source.dat				encode WZ frame source data, for check decode data is right or not (only using at debug)

stats.dat				H.264 relevant file

Decoder.bat				Decoder parameters

Encoder.bat				Encoder	parameters

wz.y					reconstructed video (decoded key and WZ frames)

AvgPSNR.log				Decode PSNR data

AvgRate.log				Decode Rate data

data.txt				H.264 relevant file

jm.log					H.264 relevant file

Decoder.exe				Decode executable file

Encoder.exe				Encode executable file

Decoder.pdb Encoder.pdb Decoder.ilk Encoder.ilk		Visual Studio file

Hall Monitor:

QP	qp	AvgRate		AvgPSNR

7	24	244.722146	40.021896

6	29	160.510377	37.04286

5	31	135.235161	35.726999

4	33	110.752249	34.233579

3	33	103.50642	33.537759

2	36	76.945433	31.649084

1	36	76.894362	31.649285

0	37	71.171437	31.230408