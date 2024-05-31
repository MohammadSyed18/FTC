.PHONY: test clean
test:
	mkdir -p build/classes
	javac -d build/classes src/main/java/dev/ftc/control/*.java src/test/java/dev/ftc/control/*.java
	java -cp build/classes dev.ftc.control.ControlTest
clean:
	rm -rf build
