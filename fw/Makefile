BINARY = main

OBJS += board.o gpsdo.o sampler.o uxb_waveform.o

OBJS += libuxb.o uxb_slave.o
OBJS += waveform.pb.o nanopb/pb_common.o nanopb/pb_encode.o nanopb/pb_decode.o

CFLAGS += -Inanopb
LDSCRIPT = stm32f303re.ld
include Makefile.include

proto:
	protoc --plugin=nanopb/generator/protoc-gen-nanopb --proto_path=. --nanopb_out=. --proto_path=nanopb/generator/proto *.proto
