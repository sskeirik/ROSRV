package rosmop.parser.ast;

public class Parameter {
    private String type;
    private String name;

    public Parameter(String type, String name) {
        this.type = type;
        this.name = name;
    }

    public String getType() {
        return type;
    }

    public String getName() {
        return name;
    }

    @Override
    public String toString() {
        return type + " " + name;
    }
}
