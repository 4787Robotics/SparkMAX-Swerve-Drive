# Setup development environment

## Install the Java Development Kit

The Deep Java Library (DJL)  project requires JDK 11 (or later). 

Verify that Java is available in your $PATH environment variable by using the following commands. If you have multiple versions of Java installed,
you can use the $JAVA_HOME environment variable to control which version of Java to use.

For ubuntu:

```bash
sudo apt-get install openjdk-11-jdk
```

For centos

```bash
sudo yum install java-11-openjdk
```

For Mac:

```bash
brew tap homebrew/cask-versions
brew update
brew install --cask temurin11
```

You can also download and install [Oracle JDK](https://www.oracle.com/technetwork/java/javase/overview/index.html)
manually if you have trouble with the previous commands.

## (Optional) Use IntelliJ 

You can use the IDE of your choice. We recommend using IntelliJ.

### Import the DJL project into IntelliJ

1. Open IntelliJ and click `Import Project`.
2. Navigate to the DJL project root folder and choose `Open`.
3. Choose `Import project from existing model` and select `Gradle`.
4. Select the default configuration and choose `OK`.

## (Optional) Import using Gradle/Maven wrappers

You use Gradle and Maven wrappers to build the project, so you don't need to install Gradle or Maven.
However, you should have basic knowledge about the Gradle or Maven build system.
