SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
DEPENDS = $(SOURCES:.cpp=.d)
LDFLAGS = $(shell pkg-config --libs gtkmm-2.4 gtkglextmm-1.2 lua5.1) -llua5.1
CPPFLAGS = $(shell pkg-config --cflags gtkmm-2.4 gtkglextmm-1.2 lua5.1)
CXXFLAGS = $(CPPFLAGS) -W -Wall -g
CXX = g++
MAIN = puppeteer

all: $(MAIN)

depend: $(DEPENDS)

clean:
	rm -f *.o *.d $(MAIN)

$(MAIN): $(OBJECTS)
	@echo Creating $@...
	@$(CXX) -o $@ $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	@echo Compiling $<...
	@$(CXX) -o $@ -c $(CXXFLAGS) $<

%.d: %.cpp
	@echo Building $@...
	@set -e; $(CC) -M $(CPPFLAGS) $< \
                  | sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
                [ -s $@ ] || rm -f $@

include $(DEPENDS)

handin: $(MAIN)
	cp *.[hc]pp ~/cs488/handin/A3/src/
	cp $(MAIN) ~/cs488/handin/A3/
	cp puppet.lua ~/cs488/handin/A3/
	cp -n ~/cs488/work/README.skel ../README
	sed -r "s/(^The final executable was compiled on this machine:)[^$$]*$$/\1 $(shell hostname)/" \
		< ../README > ~/cs488/handin/A3/README
	cp ../screenshot??.png ~/cs488/handin/A3/
	mkdir -p ../data
	cp -r ../data ~/cs488/handin/A3/
	cd ~/cs488/handin; /u/gr/cs488/bin/grsubmit A3 > ~/cs488/work/A3/checksum; cd -
