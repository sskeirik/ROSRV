package rosmop.parser.ast;

import java.util.List;

public class CFunction {
    private final String returnType;
    private final String name;
    private final List<Parameter> params;
    private final String content;

    public CFunction(String returnType, String name, List<Parameter> params, String content) {
        this.returnType = returnType;
        this.name = name;
        this.params = params;
        this.content = content;
    }

    public String getReturnType() {
        return returnType;
    }

    public String getName() {
        return name;
    }

    public List<Parameter> getParams() {
        return params;
    }

    public String getContent() {
        return content;
    }
}
