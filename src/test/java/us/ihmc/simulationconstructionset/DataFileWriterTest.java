package us.ihmc.simulationconstructionset;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataFileWriterTest
{
   private static final String TEST_DIRECTORY = "us/ihmc/simulationconstructionset/dataFileWriterTest/";

   @Test // timeout=300000
   public void testDataFileWriterAndReader() throws IOException, URISyntaxException
   {
      int numDataPoints = 10000;

      YoBuffer dataBuffer = new YoBuffer(numDataPoints);

      YoRegistry rootRegistry = new YoRegistry("rootRegistry");
      YoRegistry registryOne = new YoRegistry("registryOne");
      YoRegistry registryTwo = new YoRegistry("registryTwo");
      YoRegistry registryThree = new YoRegistry("registryThree");

      rootRegistry.addChild(registryOne);
      rootRegistry.addChild(registryTwo);
      registryTwo.addChild(registryThree);

      YoDouble variableOne = new YoDouble("variableOne", rootRegistry);
      YoDouble variableTwo = new YoDouble("variableTwo", rootRegistry);
      YoDouble variableThree = new YoDouble("variableThree", rootRegistry);
      YoDouble variableFour = new YoDouble("variableFour", registryOne);
      YoDouble variableFive = new YoDouble("variableFive", registryTwo);
      YoBoolean variableSix = new YoBoolean("variableSix", rootRegistry);
      YoInteger variableSeven = new YoInteger("variableSeven", registryThree);

      dataBuffer.addVariable(variableOne);
      dataBuffer.addVariable(variableTwo);
      dataBuffer.addVariable(variableThree);
      dataBuffer.addVariable(variableFour);
      dataBuffer.addVariable(variableFive);
      dataBuffer.addVariable(variableSix);
      dataBuffer.addVariable(variableSeven);

      for (int i = 0; i < numDataPoints - 1; i++)
      {
         variableOne.set(Math.random());
         variableTwo.set(Math.random());
         variableThree.set((int) (Math.random() * 100.0));
         variableFour.set((int) (Math.random() * 100.0));
         variableFive.set(Math.random());
         variableSix.set(Math.random() > 0.5);
         variableSeven.set((int) (Math.random() * 1000.0));

         dataBuffer.tickAndWriteIntoBuffer();
      }

      Robot robot = new Robot("testRobot");

      List<YoVariable> allVariables = rootRegistry.subtreeVariables();

      boolean binary = false;
      boolean compress = false;
      boolean spreadsheetFormatted = true;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

      spreadsheetFormatted = false;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

      binary = true;
      compress = false;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

      binary = false;
      compress = true;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

      binary = true;
      compress = true;
      testDataWriteReadIsTheSame(dataBuffer, allVariables, binary, compress, spreadsheetFormatted, robot);

   }

   private void testDataWriteReadIsTheSame(YoBuffer dataBuffer, List<YoVariable> allVariables, boolean binary, boolean compress,
                                           boolean spreadsheetFormatted, Robot robot)
         throws IOException, URISyntaxException
   {
      String filename = TEST_DIRECTORY + "testFile.data";
      if (spreadsheetFormatted)
         filename = filename + ".csv";
      if (compress)
         filename = filename + ".gz";

      URL resource = getClass().getClassLoader().getResource(filename);
      File testFile = new File(resource.getFile());

      String model = "testModel";
      double recordDT = 0.001;

      DataFileWriter dataFileWriter = new DataFileWriter(testFile);
      if (spreadsheetFormatted)
      {
         dataFileWriter.writeSpreadsheetFormattedData(dataBuffer, allVariables);

      }
      else
      {
         dataFileWriter.writeData(model, recordDT, dataBuffer, allVariables, binary, compress, robot);
      }

      DataFileReader dataFileReader = new DataFileReader(testFile);
      YoBuffer readBackBuffer = new YoBuffer(dataBuffer.getBufferSize());
      YoRegistry readBackRegistry = new YoRegistry("rootRegistry");

      YoVariableList newVars = new YoVariableList("newVars");

      dataFileReader.readData(newVars, readBackRegistry, readBackBuffer);

      boolean dataIsEqual = readBackBuffer.epsilonEquals(dataBuffer, 1e-7);
      assertTrue(dataIsEqual);
   }

   @SuppressWarnings("deprecation")

   @Test // timeout=300000
   public void testFileReadAndWriteWithDataOutputStreamAndDataInputStream() throws IOException, NullPointerException
   {
      Random rng = new Random();
      String testString = "This string tests readLine";
      double testDouble = rng.nextDouble();
      int testInteger = rng.nextInt();
      String filename = TEST_DIRECTORY + "shortReadWriteTestFile.txt";
      URL resource = getClass().getClassLoader().getResource(filename);
      File testFile = new File(resource.getFile());

      DataOutputStream outputStream = new DataOutputStream(new FileOutputStream(testFile));
      outputStream.writeDouble(testDouble);
      outputStream.writeBytes(testString + "\n");
      outputStream.writeInt(testInteger);
      outputStream.close();

      DataInputStream inputStream = new DataInputStream(new FileInputStream(testFile));
      double doubleReadBack = inputStream.readDouble();
      String lineReadBack = inputStream.readLine();
      int integerReadBack = inputStream.readInt();
      inputStream.close();

      assertTrue(testDouble == doubleReadBack);
      assertTrue(testString.equals(lineReadBack));
      assertTrue(testInteger == integerReadBack);
   }

   @Test // timeout=300000
   public void testFileReadAndWriteBackWithDataOutputStreamAndDeferredBufferedReaderCreation() throws IOException
   {
      Random rng = new Random();
      String testString = "This string tests readLine";
      double testDouble = rng.nextDouble();
      int testInteger = rng.nextInt();
      String filename = TEST_DIRECTORY + "shortReadWriteTestFile.txt";
      URL resource = getClass().getClassLoader().getResource(filename);
      File testFile = new File(resource.getFile());

      DataOutputStream outputStream = new DataOutputStream(new FileOutputStream(testFile));
      outputStream.writeDouble(testDouble);
      outputStream.writeInt(testInteger);
      outputStream.writeBytes(testString + "\n");
      outputStream.close();

      DataInputStream inputStream = new DataInputStream(new FileInputStream(testFile));
      double doubleReadBack = inputStream.readDouble();
      int integerReadBack = inputStream.readInt();

      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputStream, "US-ASCII"));
      String lineReadBack = bufferedReader.readLine();

      inputStream.close();
      bufferedReader.close();

      System.out.println(lineReadBack);
      System.out.println(testString);

      assertTrue(testDouble == doubleReadBack);
      assertTrue(testString.equals(lineReadBack));
      assertTrue(testInteger == integerReadBack);
   }

   @Test // timeout=300000
   public void testFileReadAndWriteBackWithDataOutputStreamAndBufferedReaderStringsOnly() throws IOException
   {
      String string1 = "This is the first string";
      String string2 = "This is the second string";
      String string3 = "This is the third string";
      String filename = TEST_DIRECTORY + "shortReadWriteTestFile.txt";
      URL resource = getClass().getClassLoader().getResource(filename);
      File testFile = new File(resource.getFile());

      DataOutputStream outputStream = new DataOutputStream(new FileOutputStream(testFile));
      outputStream.writeBytes(string1 + "\n");
      outputStream.writeBytes(string2 + "\n");
      outputStream.writeBytes(string3 + "\n");
      outputStream.close();

      BufferedReader reader = new BufferedReader(new InputStreamReader(new FileInputStream(testFile), "US-ASCII"));

      String readBack1 = reader.readLine();
      String readBack2 = reader.readLine();
      String readBack3 = reader.readLine();

      reader.close();

      // System.out.println("String 1: " + string1);
      // System.out.println("String 2: " + string2);
      // System.out.println("String 3: " + string3);
      //
      // System.out.println("Readback 1: " + readBack1);
      // System.out.println("Readback 2: " + readBack2);
      // System.out.println("Readback 3: " + readBack3);

      assertTrue(string1.equals(readBack1));
      assertTrue(string2.equals(readBack2));
      assertTrue(string3.equals(readBack3));
   }

   @Test // timeout = 30000
   public void testWritingAndReadingALongStateFile() throws IOException
   {
      File fileOne = new File("fileOne.state");

      if (fileOne.exists())
         fileOne.delete();

      long seed = 1776L;
      int numberOfVariables = 2000; // 12000 for when testing long files for efficiency;
      Random random = new Random(seed);
      List<YoVariable> variables = createALargeNumberOfVariables(random, numberOfVariables);
      YoVariableList originalVarList = new YoVariableList("originalVarList");
      originalVarList.addAll(variables);

      writeALongStateFile(fileOne, variables);

      DataFileReader dataFileReader = new DataFileReader(fileOne);

      YoVariableList newVarList = new YoVariableList("newVarList");
      boolean createMissingVariables = true;
      boolean printErrorForMissingVariables = false;
      YoRegistry registry = new YoRegistry("root");

      dataFileReader.readState(newVarList, createMissingVariables, printErrorForMissingVariables, registry);

      assertEquals(originalVarList.size(), newVarList.size());

      for (int i = 0; i < originalVarList.size(); i++)
      {
         YoVariable originalVariable = originalVarList.get(i);
         YoVariable newVariable = newVarList.findVariable(originalVariable.getName());

         assertFalse(originalVariable == newVariable);
         assertEquals(originalVariable.getValueAsDouble(), newVariable.getValueAsDouble(), 1e-7);

      }

      fileOne.delete();
   }

   @Test // timeout = 30000
   public void testWritingAndReadingADataFileWithLotsOfVariables() throws IOException
   {
      File fileOne = new File("fileOne.state.gz");

      if (fileOne.exists())
         fileOne.delete();

      long seed = 1776L;
      int numberOfVariables = 2000; // 12000 for when testing long files for efficiency;
      Random random = new Random(seed);
      List<YoVariable> variables = createALargeNumberOfVariables(random, numberOfVariables);
      YoVariableList originalVarList = new YoVariableList("originalVarList");
      originalVarList.addAll(variables);

      int bufferSize = 50;
      YoBuffer dataBuffer = new YoBuffer(bufferSize);

      dataBuffer.addVariables(variables);

      for (int i = 0; i < bufferSize / 2; i++)
      {
         dataBuffer.tickAndReadFromBuffer(1);
      }

      dataBuffer.setInOutPointFullBuffer();

      Robot robot = new Robot("testWritingRobot");
      writeALongDataFile(fileOne, dataBuffer, variables, robot);

      System.out.println("Wrote File. Now reading it.");

      DataFileReader dataFileReader = new DataFileReader(fileOne);

      YoVariableList newVarList = new YoVariableList("newVarList");
      YoRegistry registry = new YoRegistry("rootRegistry");

      YoBuffer newDataBuffer = new YoBuffer(16384);
      dataFileReader.readData(newVarList, registry, newDataBuffer);

      assertEquals(originalVarList.size(), newVarList.size());

      for (int i = 0; i < originalVarList.size(); i++)
      {
         YoVariable originalVariable = originalVarList.get(i);
         YoVariable newVariable = newVarList.findVariable(originalVariable.getName());

         assertFalse(originalVariable == newVariable);
         assertEquals(originalVariable.getValueAsDouble(), newVariable.getValueAsDouble(), 1e-7);

      }

      fileOne.delete();
   }

   private void writeALongStateFile(File file, List<YoVariable> variables)
   {
      DataFileWriter dataFileWriter = new DataFileWriter(file);

      boolean compress = false;
      double recordDT = 0.001;
      boolean binary = false;
      dataFileWriter.writeState("model", recordDT, variables, binary, compress);
   }

   private void writeALongDataFile(File file, YoBuffer dataBuffer, List<YoVariable> variables, Robot robot)
   {
      DataFileWriter dataFileWriter = new DataFileWriter(file);

      boolean compress = true;
      double recordDT = 0.001;
      boolean binary = true;
      dataFileWriter.writeData("model", recordDT, dataBuffer, variables, binary, compress, robot);
   }

   private List<YoVariable> createALargeNumberOfVariables(Random random, int numberOfVariables)
   {
      YoRegistry rootRegistry = new YoRegistry("rootRegistry");
      YoRegistry registryOne = new YoRegistry("registryOne");
      YoRegistry registryTwo = new YoRegistry("registryTwo");
      YoRegistry registryThree = new YoRegistry("registryThree");

      rootRegistry.addChild(registryOne);
      registryOne.addChild(registryTwo);
      registryTwo.addChild(registryThree);

      YoDouble t = new YoDouble("t", registryThree);
      YoDouble time = new YoDouble("time", registryThree);
      t.set(1.1);
      time.set(2.2);

      for (int i = 0; i < numberOfVariables; i++)
      {
         YoDouble variable = new YoDouble("variable" + i, registryThree);
         variable.set(Math.random());
      }

      return rootRegistry.subtreeVariables();
   }

}
