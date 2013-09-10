
CPP	=	g++ -O3 -Wdeprecated -fpermissive -fexceptions 
INCPATH  = /usr/local/inc 
LIBPATH = /usr/local/lib
SYSLIBSOCC	=	-lrt -lTKMath -lTKXSBase -lTKernel -ldl -lTKIGES -lTKSTEP -lTKBRep -lTKMesh
PRGS	=	occdaemon

occdaemon:  Lib/occdaemon.o occdaemon.cpp
	$(CPP) -fexceptions -D_BOOL  $< -I$(INCPATH) -L$(LIBPATH) -DHAVE_IOMANIP -DHAVE_IOSTREAM -DHAVE_LIMITS_H  $(SYSLIBSOCC) -o $@

clean:
	rm -rf Lib/*;\
	rm -rf MayaPlug/*;

Lib/occdaemon.o: occdaemon.cpp vect.h
	$(CPP) -fexceptions -D_BOOL -c $< -I$(INCPATH) -L$(LIBPATH)  -DHAVE_IOMANIP -DHAVE_IOSTREAM -DHAVE_LIMITS_H $(SYSLIBSOCC) -o $@

#Lib/SAX2ParseHandler.o: SAX2ParseHandler.cpp SAX2ParseHandler.h VertexCoordArray.h Sets.h DAG.h PolyMesh.h Dispatcher.h DAG.cpp




