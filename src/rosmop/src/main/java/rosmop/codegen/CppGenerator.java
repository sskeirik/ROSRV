package rosmop.codegen;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.ArrayList;
import java.util.List;
import java.util.Comparator;

import org.apache.commons.io.FilenameUtils;

import rosmop.ROSMOPException;
import rosmop.RVParserAdapter;
import rosmop.parser.ast.ROSEvent;
import rosmop.parser.ast.Variable;
import rosmop.util.MessageParser;
import rosmop.util.Tool;

import com.runtimeverification.rvmonitor.c.rvc.CSpecification;
import com.runtimeverification.rvmonitor.logicpluginshells.LogicPluginShellResult;
import com.runtimeverification.rvmonitor.core.ast.Event;

/**
 * @author Cansu Erdogan
 *
 * Generates rvmonitor.cpp file from the wrapped ASTs of input specification files
 *
 */
public class CppGenerator {

    protected static SourcePrinter printer = new SourcePrinter();

    public static Set<String> getTopics() {
        return HeaderGenerator.addedTopics.keySet();
    }

    public static String getMessageTypeForTopic(String topic) {
        return HeaderGenerator.addedTopics.get(topic).get(0).classifyMsgType();
    }

    public static Variable getParameterForEvent(Event e) {
        ROSEvent event = (ROSEvent) e;
        if (event.getParameters() == null) return null;
        assert event.getParameters().size() == 1 : "ROS Events must have 1 parameters.";
        return event.getParameters().get(0);
    }

    public static String getTopicForEvent(Event e) {
        ROSEvent event = (ROSEvent) e;
        return event.getTopic();
    }

    public static String callbackNameForEvent(Event e) {
        return "callback_" + e.getName();
    }

    public static String accessorNameForEvent(Event e) {
        return "accessor_" + e.getName();
    }

    public static String getPatternForParameter(Event e, Variable v) {
        ROSEvent event = (ROSEvent) e;
        return event.getPattern().get(v.getDeclaredName());
    }

    public static String getCNameForTopic(String topic) {
        if (topic.charAt(0) == '/')
        topic = topic.substring(1);
        return topic;
    }

    public static List<Event> getEventsForCSpecification(CSpecification cspec) {
        List<Event> events = new ArrayList(cspec.getEvents().values());
        events.sort(Comparator.comparing(x -> x.getName()));
        return events;
    }

    public static void generateCpp(HashMap<CSpecification, LogicPluginShellResult> toWrite,
                                   String outputPath, boolean monitorAsNode)
        throws FileNotFoundException, ROSMOPException, java.io.IOException
    {
        String headerFile = FilenameUtils.getBaseName(outputPath);

        printer.printLn("#include \"" + headerFile + ".h\"");
        printer.printLn("#include <rv/monitor.h>");

        for (CSpecification cspec : toWrite.keySet()) {
            printer.printLn(cspec.getIncludes());
        }

        printer.printLn();
        printer.printLn("using namespace std;");
        printer.printLn("namespace rosmop_generated");
        printer.printLn("{"); printer.indent();

        for (CSpecification cspec : toWrite.keySet()) {
            printer.printLn("struct " + cspec.getSpecName());
            printer.printLn("{"); printer.indent();

            printer.printLn(cspec.getDeclarations());
            printer.printLn();

            /*
            // TODO: What are these?
            if (toWrite.get(cspec) != null) {
                printer.printLn((String) toWrite.get(cspec).properties.get("state declaration"));
                printer.printLn((String) toWrite.get(cspec).properties.get("categories"));
                printer.printLn((String) toWrite.get(cspec).properties.get("monitoring body"));
                printer.printLn();
            }
            */

            for (Event event : getEventsForCSpecification(cspec)) {
                printer.printLn("/* " + event.getName() + " */");
                Variable param = getParameterForEvent(event);
                printer.printLn("void " + callbackNameForEvent(event) + "(" +
                                    param.getType() + " " + param.getDeclaredName() +  ")");
                printer.printLn(event.getAction());
                printer.printLn(param.getType()  + " " + accessorNameForEvent(event) + "(" + getMessageTypeForTopic(getTopicForEvent(event)) + "& message)" );
                printer.printLn("{ return message." +  getPatternForParameter(event, param) + "; }");
            }

            printer.unindent(); printer.printLn("};");
        }

        printer.printLn();
        printer.printLn("struct Monitor");
        printer.printLn("{");
        printer.indent();
        printer.printLn("ros::NodeHandle n;");

        printer.printLn();
        for (String topic : getTopics()) {
            printer.printLn("rv::monitor::MonitorTopic<" + getMessageTypeForTopic(topic) + "> " + getCNameForTopic(topic) + ";");
        }

        printer.printLn();
        for (CSpecification cspec : toWrite.keySet()) {
            printer.printLn("rosmop_generated::" + cspec.getSpecName() + " " + cspec.getSpecName() + ";");
        }

        printer.printLn();
        printer.printLn("Monitor()");
        for (String topic : getTopics()) {
            printer.printLn("    : " + getCNameForTopic(topic) + "(n, \"" + topic + "\", 1000)");
        }
        printer.printLn("{"); printer.indent();
        for (CSpecification cspec : toWrite.keySet()) {
            for (Event event : getEventsForCSpecification(cspec)) {
                String topic = getTopicForEvent(event);

                printer.printLn(getCNameForTopic(topic) + ".registerEvent(" +
                       "&" + cspec.getSpecName() +
                   ", &" + "rosmop_generated::" + cspec.getSpecName() + "::" + callbackNameForEvent(event) +
                   ", &" + "rosmop_generated::" + cspec.getSpecName() + "::" + accessorNameForEvent(event) +
                      ");");
            }
        }
        printer.unindent(); printer.printLn("}");
        printer.unindent(); printer.printLn("};");
        printer.unindent(); printer.printLn("}");

        printer.printLn();
        printer.printLn("int main(int argc, char ** argv)\n"
                      + "{\n"
                      + "    ros::init(argc, argv, \"rvmonitor\");\n"
                      + "    rosmop_generated::Monitor m;\n"
                      + "    ros::spin();\n"
                      + "    return 0;\n"
                      + "}");

        Tool.writeFile(printer.getSource(), outputPath);
    }

}
