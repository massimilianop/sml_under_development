"""This module implements the class Jsonable."""


import json
import inspect
import numpy as np


class Jsonable:
    """A Jsonable object is an object that can be constructed
    from a json string.
    In the sml world, trajectories, simulators and controllers are Jsonable.
    If a Jsonable contains nested Jsoable objects,
    those should be declared in the class variable `inner`.
    """


    inner = dict()
    """This is the only object that needs to be redefined by the children.
    Each key is one of the arguments in the constructor that is itself a
    Jsonable. The corresponding value is a dictionary, which contains the
    possible class names for that argument.
    For example, suppose this was a QuadController class, containing a
    db_int_con object. Suppose that the db_int_con can be of tipes PCon, PICon
    and PIDCon.
    Then we have `QuadController.inner = {'db_int_con': {"PCon": PCon,
    "PICon": PICon, "PIDCon", PIDCon}}.
    """
    
    @classmethod
    def to_string(cls, inner=dict()):
        """Returns a string
        that can be used to construct an object of this class.
        In the `inner` dictionary, each key is one of the arguments of the
        constructor that are Jsonable objects. (Therefore inner has the same keys
        as cls.inner.) The corresponding value is a 
        the chosen class name for that object.
        """
        
        spec = inspect.getargspec(cls.__init__)
        args = spec.args
        defs = spec.defaults
        arg_dic = dict()
        max_length = 0
        for i in range(len(defs)):
            arg = args[i+1]
            la = len(arg)
            if la > max_length:
                max_length = la
            if arg in cls.inner.keys():
                val = (inner[arg].__name__, inner[arg].to_string())
            elif type(defs[i]) is np.ndarray:
                val = str(list(defs[i]))
            else:
                val = defs[i]
            arg_dic[arg] = val
        # string = json.dumps(arg_dic, indent=4, separators=(', ', ':\n\t'))
        string = json.dumps(arg_dic, separators=(', \n', '\t:\t'))
        string = string.replace('"[','[')
        string = string.replace(']"',']')
        string = string.replace('{','{\n')
        string = string.replace('}','\n}')
        return string
        
        
    @classmethod
    def from_string(cls, string=""):
        """Returns an object of this class constructed from the json string
        `string`.
        """
        
        arg_dic = json.loads(string)
        for key, value in arg_dic.items():
            if key in cls.inner.keys():
                InnerObjType = cls.inner[key][value[0]]
                inner_obj = InnerObjType.from_string(value[1])
                arg_dic[key] = inner_obj
        return cls(**arg_dic)
        
        
        
#    def __init__(self, arg1=1, arg2=2, arg3=3):
#        pass
        
        
        
        

#string = Jsonable.to_string()
#print string
#jsn = Jsonable.from_string(string)
#print jsn


#class Inner(Jsonable):
#    
#    def __init__(self, arg1=1):
#        pass
#        
#        

#class Outer(Jsonable):

#    inner = {"sub": {"Inner": Inner}}
#    
#    def __init__(self, arg2=2, sub=Inner()):
#        pass
#        
#        
#        
#string = Inner.to_string()
#print string
#sub = Inner.from_string(string)
#print sub
#string = Outer.to_string(inner={'sub':Inner})
#print string
#boss = Outer.from_string(string)
#print boss
