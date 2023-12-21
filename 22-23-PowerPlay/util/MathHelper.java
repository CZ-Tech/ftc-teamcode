package org.firstinspires.ftc.teamcode.util;

import java.util.NoSuchElementException;

public class MathHelper<T extends Number>{
    public T min(T... para){
        T minvalue=para[0];
        for(T value: para)
            if(compare(minvalue,value,CompareType.Max))
                minvalue=value;
        return minvalue;
    }
    public T max(T... para){
        T maxvalue=para[0];
        for(T value: para)
            if(compare(maxvalue,value,CompareType.Min))
                maxvalue=value;
        return maxvalue;
    }
    private boolean compare(T firstvalue,T secondvalue,CompareType type){
        //不建议碰这一段代码，因为我也不知道会出啥事
        switch (type){
            case Min:{
                if(firstvalue instanceof Integer)
                    return (Integer)firstvalue<(Integer)secondvalue;
                if(firstvalue instanceof Long)
                    return (Long)firstvalue<(Long)secondvalue;
                if(firstvalue instanceof Double)
                    return (Double)firstvalue<(Double)secondvalue;
                if(firstvalue instanceof Float)
                    return (Float)firstvalue<(Float)secondvalue;
                return false;
            }
            case Max:{
                if(firstvalue instanceof Integer)
                    return (Integer)firstvalue>(Integer)secondvalue;
                if(firstvalue instanceof Long)
                    return (Long)firstvalue>(Long)secondvalue;
                if(firstvalue instanceof Double)
                    return (Double)firstvalue>(Double)secondvalue;
                if(firstvalue instanceof Float)
                    return (Float)firstvalue>(Float)secondvalue;
                return false;
            }
            case Equal:{
                if(firstvalue instanceof Integer)
                    return (Integer)firstvalue==(Integer)secondvalue;
                if(firstvalue instanceof Long)
                    return (Long)firstvalue==(Long)secondvalue;
                if(firstvalue instanceof Double)
                    return (Double)firstvalue==(Double)secondvalue;
                if(firstvalue instanceof Float)
                    return (Float)firstvalue==(Float)secondvalue;
                return false;
            }
            case NotEqual:{
                if(firstvalue instanceof Integer)
                    return (Integer)firstvalue!=(Integer)secondvalue;
                if(firstvalue instanceof Long)
                    return (Long)firstvalue!=(Long)secondvalue;
                if(firstvalue instanceof Double)
                    return (Double)firstvalue!=(Double)secondvalue;
                if(firstvalue instanceof Float)
                    return (Float)firstvalue!=(Float)secondvalue;
                return false;
            }
            case MinEqual:{
                if(firstvalue instanceof Integer)
                    return (Integer)firstvalue<=(Integer)secondvalue;
                if(firstvalue instanceof Long)
                    return (Long)firstvalue<=(Long)secondvalue;
                if(firstvalue instanceof Double)
                    return (Double)firstvalue<=(Double)secondvalue;
                if(firstvalue instanceof Float)
                    return (Float)firstvalue<=(Float)secondvalue;
                return false;
            }
            case MaxEqual:{
                if(firstvalue instanceof Integer)
                    return (Integer)firstvalue>=(Integer)secondvalue;
                if(firstvalue instanceof Long)
                    return (Long)firstvalue>=(Long)secondvalue;
                if(firstvalue instanceof Double)
                    return (Double)firstvalue>=(Double)secondvalue;
                if(firstvalue instanceof Float)
                    return (Float)firstvalue>=(Float)secondvalue;
                return false;
            }
            default:return false;
        }
    }
    private enum CompareType{
        Min,Max,Equal,NotEqual,MinEqual,MaxEqual
    }
}
