include $(BASEDIR)/script/common.mk
include $(BASEDIR)/script/include.mk
CFLAGS+=-std=c++14 -fpermissive
CURPATH := ./
INC_PATH +=  -I $(CURPATH)/../inc
INC_PATH +=  -I $(CURPATH)/../

C_SRC = $(wildcard  ./*.c)
CPP_SRC = $(wildcard  ./*.cpp)

OBJS += $(C_SRC:.c=.o)
OBJS += $(CPP_SRC:.cpp=.o)
CPPFLAGS +=-Wno-subobject-linkage
$(OBJDIR)/%.o:%.c
	$(CPP) $(CFLAGS) -c $< $(INC_PATH) -o $@

$(OBJDIR)/%.o:%.cpp
	$(CPP) $(CFLAGS) -c $< $(INC_PATH) -o $@

TARGET=$(BASEDIR)/extlib/$(BUILDTYPE)/libjson.so

OBJLIST = $(addprefix $(OBJDIR)/,$(OBJS))
$(TARGET) : $(OBJLIST)
	$(LD) $(LDFLAGS) $^ -o $@
	#$(AR) rcs $@ $^ 
	#cp $@ $(SRC_BASE)/libs 

all:$(TARGET)
	echo "finished"

clean: 
	RM $(OBJDIR) $(TARGET)

