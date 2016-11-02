"""This module implements the class Jsonable."""


import json
import inspect
import numpy as np

import collections

# dictionary = {
#     'bed'   :['Bed1', {'pillow':[],'doll':[]}],
#     'lamp'  :['Lamp1', {}]
#     }


def check_completeness(dictionary):
    if any([len(value)==0 for value in dictionary.values()]):
        return False
    else:
        return all([check_completeness(value[1]) for value in dictionary.values()])

# print check_completeness(dictionary)

def update_input_dictionary(dictionary, list_of_keys, NestedClassName, nested_dictionary):
    inner_dictionary = dict(dictionary)
    for key in list_of_keys[0:-1]:
        inner_dictionary = dict(inner_dictionary[key][1])
    empty_list = inner_dictionary[list_of_keys[-1]]
    empty_list.append(NestedClassName)
    empty_list.append(nested_dictionary)
    
# update_input_dictionary(dictionary, ['bed', 'doll'], 'Doll1', {})
#print dictionary 
#print check_completeness(dictionary)

def add_item_to_database(dic,Class,default=False):
    dic[Class.__name__] = Class
    if default:
        dic["Default"] = Class


# # this is a decorator
# def add_to_database(Class):

#     frame = inspect.currentframe()
#     local_variables = frame.f_back.f_locals

#     if 'database' in local_variables.keys():
#         local_variables['database'][Class.__name__] = Class
#     else:
#         # create database
#         local_variables['database'] = {Class.__name__:Class}
    
#     # return Class
#     return Class
# this is a decorator
def add_to_database(database_name='database',default=False):

    frame = inspect.currentframe()
    local_variables = frame.f_back.f_locals

    def decorator(Class):

        if database_name in local_variables.keys():
            local_variables[database_name][Class.__name__] = Class
        else:
            # create database
            local_variables[database_name] = {Class.__name__:Class}
        
        if default:
            local_variables[database_name]['Default'] = Class

        # return Class
        return Class

    return decorator

# this is a decorator
def set_default_in_database(Class):

    frame = inspect.currentframe()
    local_variables = frame.f_back.f_locals

    if 'database' in local_variables.keys():
        local_variables['database']['Default'] = Class
    else:
        print('ERROR: NO DATABASE')
    
    # return Class
    return Class

class Jsonable:
    """A Jsonable object is an object that can be constructed
    from a json string.
    In the sml world, trajectories, simulators and controllers are Jsonable.
    If a Jsonable contains nested Jsoable objects,
    those should be declared in the class variable `inner`.
    """

    UNIQUE_STRING = "##########"

    def __init__(self):
        self.add_inner_defaults()
        return

    @classmethod
    def get_dic_recursive(cls, dictionary, list_of_keys):
        CurrentClass = cls
        #print CurrentClass
        current_dictionary = dict(dictionary)
        #print current_dictionary
        current_inner = dict(cls.inner)
        #print current_inner
        for key in list_of_keys[0:-1]:
            CurrentClassName = current_dictionary[key][0]
            #print CurrentClassName
            CurrentClass = current_inner[key][CurrentClassName]
            #print CurrentClass
            current_inner = dict(CurrentClass.inner)
            #print current_inner
            current_dictionary = dict(current_dictionary[key][1])
            #print current_dictionary
        #print CurrentClass.inner[list_of_keys[-1]]
        return CurrentClass.inner[list_of_keys[-1]]
            


    @classmethod
    def contained_objects(cls):
        return cls.inner


    inner = collections.OrderedDict()
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

        if defs is None:
            defs = []
        arg_dic = dict()
        
        for i in range(len(defs)):
            arg = args[i+1]
            if type(defs[i]) is np.ndarray:
                val = str(list(defs[i]))
            else:
                val = defs[i]
            arg_dic[arg] = val

        for arg in cls.inner.keys():
            inner_key = inner[arg][0]
            inner_inner = inner[arg][1]
            val = (inner_key, json.loads(cls.inner[arg][inner_key].to_string(inner_inner)))
            arg_dic[arg] = val

        # string = json.dumps(arg_dic)
        # string = json.dumps(arg_dic, indent=4, separators=(', ', ':\n\t'))
        string = json.dumps(arg_dic, separators=(', \n', '\t: '))
        string = string.replace('"[','[')
        string = string.replace(']"',']')
        string = string.replace('{','{\n')
        string = string.replace('}','\n}')
        for i in range(10):
            string = string.replace(', \n'+str(i),', '+str(i))
            string = string.replace(', \n'+str(-i),', '+str(-i))
        # string = string.replace(', \n0',', 0')
        # string = string.replace(', \n1',', 1')
        # string = string.replace(', \n2',', 2')
        # string = string.replace(', \n3',', 3')
        # string = string.replace(', \n4',', 4')
        # string = string.replace(', \n5',', 5')
        # string = string.replace(', \n6',', 6')
        # string = string.replace(', \n7',', 7')
        # string = string.replace(', \n8',', 8')
        # string = string.replace(', \n9',', 9')

        #string = string.replace('"','')
        return string
        
    def from_object_to_string(self):
        """Returns a string that can be used to REconstruct the object.
        Unlike the to_string method, no inner is neccessary
        """
        
        spec = inspect.getargspec(self.__init__)
        args = spec.args 
        defs = spec.defaults

        if defs is None:
            defs = []
        arg_dic = dict()
        
        for i in range(len(defs)):
            arg = args[i+1]
            if arg in self.inner.keys():
                inner_key   = inner[arg][0]
                inner_inner = inner[arg][1]
                val         = (inner_key, json.loads(self.inner[arg][inner_key].from_object_to_string(inner_inner)))
            elif type(defs[i]) is np.ndarray:
                val = str(list(defs[i]))
            else:
                val = defs[i]
            arg_dic[arg] = val

        string = json.dumps(arg_dic)

        return string

        
    # @classmethod
    # def from_string(cls, string=""):
    #     """Returns an object of this class constructed from the json string
    #     `string`.
    #     """
        
    #     arg_dic = json.loads(string)
    #     for key, value in arg_dic.items():
    #         if key in cls.inner.keys():
    #             InnerObjType = cls.inner[key][value[0]]
    #             inner_obj = InnerObjType.from_string(json.dumps(value[1]))
    #             arg_dic[key] = inner_obj
    #     return cls(**arg_dic)
        
    @classmethod
    def from_string(cls, string=""):
        """Returns an object of this class constructed from the json string
        `string`.

        obj.constructing_dic is "global": if innner changes, this dictionary changes
        """

        if not string == "":
            arg_dic = json.loads(string)

            arg_dic_object = dict()
            spec = inspect.getargspec(cls.__init__)
            # first argument in __init__ is self: disregard this
            for arg in spec.args[1:]:
                # if init is changed, arguments may differ
                if arg in arg_dic.keys():
                    arg_dic_object[arg] = arg_dic[arg]
                else:
                    print('Using default value for '+str(arg)+' in class '+cls.__name__)
                    # no need to populate arg_dic_object[arg], since default will be used

            # construct object with default inner objects
            obj = cls(**arg_dic_object)
            obj.constructing_dic = obj.get_constructing_dic()

            # change inner objects
            for inner_key in cls.inner.keys():
                [key,constructing_dic] = arg_dic[inner_key]
                # signature change_inner_key(self,inner_key,key,input_string)
                dictionary_of_classes = cls.inner[inner_key]
                Class  = dictionary_of_classes[key]
                class_object = Class.from_string(json.dumps(constructing_dic))
                setattr(obj,inner_key,class_object)
                obj.constructing_dic[inner_key] = [key,class_object.constructing_dic]

        else:
            "if string is empty, construct default object"
            print("\nYou are constructing a default object of class: " + cls.__name__+'\n')
            obj = cls()

        return obj        

    def get_constructing_dic(self):
        
        dictionary = dict()

        spec = inspect.getargspec(self.__init__)
        args = spec.args
        defs = spec.defaults

        if defs is None:
            defs = []

        dictionary = dict()
        
        for i in range(len(defs)):
            arg = args[i+1]

            if hasattr(self,arg):
                dictionary[arg] = getattr(self,arg)
            else:
                print("BEWARE: "+arg+" is not attribute of "+self.__class__.__name__)
                dictionary[arg] = defs[i]
            
        return dictionary


    def get_constructing_string(self):
        """Returns string that constructs object.
        """

        if hasattr(self,"constructing_dic"):

            spec = inspect.getargspec(self.__init__)

            arg_dic = dict()

            # print(self.constructing_dic)
            for arg in spec.args[1:]:
                if type(self.constructing_dic[arg]) is np.ndarray:
                    val = str(list(self.constructing_dic[arg]))
                else:
                    val = self.constructing_dic[arg]
                arg_dic[arg] = val

            for arg in self.inner.keys():
                val = (self.constructing_dic[arg][0], json.loads(getattr(self,arg).get_constructing_string()))
                arg_dic[arg] = val

            constructing_string = json.dumps(arg_dic, separators=(', \n', '\t: '))
            constructing_string = constructing_string.replace('"[','[')
            constructing_string = constructing_string.replace(']"',']')
            constructing_string = constructing_string.replace('{','{\n')
            constructing_string = constructing_string.replace('}','\n}')            

        else:
            constructing_string = ""

        # print(constructing_string)
        return constructing_string

    # def get_constructing_string(self):
    #     """Returns string that constructs object.
    #     """

    #     spec = inspect.getargspec(self.__init__)
    #     args = spec.args 
    #     defs = spec.defaults

    #     if defs is None:
    #         defs = []
    #     arg_dic = dict()
        
    #     for i in range(len(defs)):

    #         arg = args[i+1]
    #         if hasattr(self,arg):
    #             val = self.arg
    #         else:
    #             print(str(arg)+" is not an attribute of "+self.__class__.__name__)
    #             if type(defs[i]) is np.ndarray:
    #                 val = str(list(defs[i]))
    #             else:
    #                 val = defs[i]

    #         arg_dic[arg] = val

    #     for arg in self.inner.keys():
    #         inner_arg_string = getattr(self,arg).get_constructing_string()
    #         inner_arg_dic    = json.loads(inner_arg_string)
    #         class_name       = getattr(self,arg).__class__.__name__
    #         arg_dic[arg] = [class_name,json.loads(inner_arg_dic)]

    #     return json.dumps(arg_dic)        

    @classmethod
    def add_inner(cls,key,dictionary):
        """Add (key,dictionary) to locals"""

        if isinstance(dictionary,dict):

            frame = inspect.currentframe()
            local_variables = frame.f_back.f_locals
            if 'inner' in local_variables.keys(): 
                local_variables['inner'][key] = dictionary
            else:
                local_variables['inner'] = collections.OrderedDict()
                local_variables['inner'][key]  = dictionary
        else:
            print("ERROR: you must provide a dictionary")

    def add_inner_defaults(self):
        """Add key attribute, and the default object associated
        to that attribute
        """

        for key,dictionary in self.inner.items():
            setattr(self,key,dictionary['Default']())

    def change_inner_key(self,inner_key,key,input_string):

        if hasattr(self,inner_key):
            # inner = {inner_key : dictionary_of_classes}
            dictionary_of_classes = self.inner[inner_key]

            if key in dictionary_of_classes.keys():
                Class  = dictionary_of_classes[key]
                class_object = Class.from_string(input_string)
                setattr(self,inner_key,class_object)
                # TODO: there should always exist constructing_dic
                if hasattr(self,"constructing_dic"):
                    self.constructing_dic[inner_key] = [key,class_object.get_constructing_dic()]
            return

        else:
            print("WARNING: " + self.__class__.__name__ + "has no" + inner_key)
            return

    def change_inner_of_inner(self,sequence_inner_key,key,input_string):
        
        inner_key = sequence_inner_key.pop(0)

        if sequence_inner_key:
            if hasattr(self,inner_key): 
                getattr(self,inner_key).change_inner_of_inner(sequence_inner_key,key,input_string)
            else:
                print("WARNING: " + self.__class__.__name__ + "has no" + inner_key)
            return
        else:
            self.change_inner_key(inner_key,key,input_string)
            return
    
    def call_method_inner_of_inner(self,sequence_inner_key,func_name,input_to_func):
        # dictionary = '{"sequence_inner_key":"","func_name":"","input_to_func":""}'

        if sequence_inner_key:
            inner_key = sequence_inner_key.pop(0)
            getattr(self,inner_key).call_method_inner_of_inner(sequence_inner_key,func_name,input_to_func)
        else:
            # if sequence is empty, method is of self
            getattr(self,func_name)(**input_to_func)

    def get_parameters(self):
        """Child classes redefine this
        and return a dictionary of their parameters,
        for example: proportional_gain, derivative_gain
        """
        params = {}
        #params['param_name'] = param_value
        #...
        return params
        
        
    # def get_parameters_recursive(self):
    #     params = {}
    #     for name, obj in self.get_inner_jsonables.items():
    #         params[name] = obj.parameters_to_string_recursive()
    #     params.update(self.get_parameters())
    #     return params
        

    def get_parameters_recursive(self):
        params = {}
        for name, obj in self.get_inner_jsonables.items():
            params[name] = obj.parameters_to_string_recursive()
        params.update(self.get_parameters())
        return params

    
    #TODO make this based on the inner,
    #so that the child classes do not have to redefine it.
    def get_inner_jsonables(self):
        """Child classes redefine this
        and return the inner jsonable objects
        """
        inner = {}
        #inner[object_name] = object
        #...
        return inner
        
        
    def parameters_to_string(self):
        # dic = self.get_parameters_recursive()
        # string = json.dumps(dic)
        string = self.description()
        return string
    
    def parametric_description(self,name):

        dictionary = {}
        dictionary['name']      = name
        dictionary['to_string'] = self.get_constructing_string()

        string  = self.UNIQUE_STRING+"\n"
        string += json.dumps(dictionary)
        string += self.UNIQUE_STRING
        return string

    @classmethod
    def inverse_parametric_description(cls,string):
        """from a string withouth the unique string, get name"""
        
        # print(string)
        dictionary = json.loads(string)
        name   = dictionary['name']
        parametric_description = dictionary['to_string']
        # print(parametric_description)
        return name, parametric_description
     
        
    @classmethod
    def combined_description(cls, inner=dict()):
        """Returns a string of the combined description of all jsonable classes (class + its inners)"""
        
        spec = inspect.getargspec(cls.__init__)
        args = spec.args
        defs = spec.defaults

        if defs is None:
            defs = []
        arg_dic = dict()

        string = "<p>"+cls.description()+"</p>"

        if len(defs) != 0:
            # start list
            string += "<ul>"
            # for every inner we have a item list, and a description, since an inner is a jsonable object
            for i in range(len(defs)):
                arg = args[i+1]
                if arg in cls.inner.keys():
                    inner_key   = inner[arg][0]
                    inner_inner = inner[arg][1]
                    string += "<li>"+arg+"="+inner_key+": "+cls.inner[arg][inner_key].combined_description(inner_inner)+"</li>"
            # end list
            string += "</ul>"

        return string

    
    def object_combined_description(self):
        """Returns a string of the combined description of all jsonable objects (class + its inners)"""

        if hasattr(self,"description"):
            string = "<p>"+self.description()+"</p>"
        else:
            string = self.__class__.__name__+" has no method description(). IMPLEMENT IT"
            print(string)
            print("Please implement it\n")
        
        if hasattr(self,"object_description"):
            string+= "<p>"+self.object_description()+"</p>"
        else:
            string = self.__class__.__name__+" has no method object_description(). IMPLEMENT IT"
            print(string)
            print("Please implement it\n")

        string += "<ul>"
        for arg in self.inner.keys():
            if hasattr(self,arg):
                print(arg)
                string += "<li>"+arg+"="+getattr(self, arg).__class__.__name__+": "+getattr(self, arg).object_combined_description()+"</li>"
            else:
                print(self.__class__.__name__+" has no attribute "+arg)
        # end list
        string += "</ul>"

        return string

    def get_data(self):
        '''return list of "numbers" that should be saved'''
        # by default empty list is returned
        # i.e., nothing is saved by default
        return []

    @classmethod
    def get_data_size(self):
        '''return how many numbers are saved: this must be static'''
        return 0

    def get_complete_data(self):
        '''this get the data from object and all its inners'''
        out = self.get_data()
        for arg in self.inner.keys():
            out+=getattr(self,arg).get_complete_data()

        return out

    def plot_from_string(self,string,starting_point=[0]):
        '''Take saved data and return a tuple of figures from that data
        starting point corresponds to point where data is specific to the object
        '''
        # empty tuple of figures
        return ()

    def complete_plot_from_string(self,string,starting_point=[0]):
        '''Take saved data and return a tuple of figures 
        from object and all its inners'''

        # starting_point is list, thus is mutable: passed by reference!
        figures = self.plot_from_string(string,starting_point=starting_point)
        starting_point[0] += self.get_data_size()
        # TODO: the order of the inners may be different from get_complete_data
        # changed inner from dictionary to ordered dictionary
        for arg in self.inner.keys():
            figures+=getattr(self,arg).complete_plot_from_string(string,starting_point=starting_point)
            starting_point[0] += getattr(self,arg).get_data_size()

        return figures


###############
# TESTING
###############

# Comment out all that follows for actual use of this module.

#class Pillow(Jsonable):
#    
#    def __init__(self, is_soft=True):
#        pass
#        
#        

#class Bed(Jsonable):

#    inner = {
#        'pillow': {'Pillow1': Pillow, 'Pillow2': Pillow},
#        'cushion': {'Cushion1': Pillow, 'Cushion2': Pillow}
#        }
#    
#    def __init__(self, length=2.0, pillow=Pillow()):
#        pass



#class Room(Jsonable):

#    inner = {'bed': {'Bed1': Bed, 'Bed2': Bed}}
#    
#    def __init__(self, area=18.0, bed=Bed()):
#        pass




# dictionary = {'bed': ['Bed1', {'pillow': []}]}
# list_of_keys = ['bed', 'pillow']
# Room.get_dic_recursive(dictionary, list_of_keys)


## TODO This function must be moved to the GUI
## but can be used in the GUI as it is.
## (Apart from the print statement, which should be substituted to some message
## printed on the GUI.)
#def __construct_inner(top_dictionary, top_key):

#    inner = {}
#    TopClass = top_dictionary[top_key]
#    for param_name, param_dict in TopClass.inner.items():
#        print "A " + top_key + " contains a parameter called '" + param_name + "', which can be any of the following types. Please choose the type that you want for the parameter '" + param_name + "'."
#        key = __get_key_from_user(param_dict)
#        inner[param_name] = []
#        inner[param_name].append(key)
#        inner[param_name].append(__construct_inner(param_dict, key))
#    return inner
#    

##TODO This function must be moved to the GUI and modified.
## Instead of printing the key on the terminal, they must be printed on a GUI box.
## Then, instead of getting the user input from the terminal, the input is
## obtained by detecting which key the user has clicked on.
#def __get_key_from_user(dictionary):
#    for key in dictionary.keys():
#        print key
#    return raw_input('Your choice: ')




#print "Choose the type of room that you want."
#top_dictionary = {'Room1': Room, 'Room2': Room, 'Room3': Room}
#top_key = __get_key_from_user(top_dictionary)
#inner = __construct_inner(top_dictionary, top_key)
#print inner
#string = Room.to_string(inner)
#print string
#my_room = Room.from_string(string)
#print my_room



import inspect

def inherit_methods_list_from_parents(Class):
    """This is a decorator that takes a Class, it returns the same class
    and it appends the parents methods_list to the child methods_list
    """

    # frame = inspect.currentframe()
    # local_variables = frame.f_back.f_locals

    ParentClasses = Class.__bases__

    for ParentClass in ParentClasses:

        if hasattr(Class,"methods_list"):
            if hasattr(ParentClass,"methods_list"):
                # Class would inherit methods_list from parent, 
                # but nmethods_list is now property of class
                Class.methods_list += ParentClass.methods_list
        else:
            if hasattr(ParentClass,"methods_list"):
                # Class would inherit methods_list from parent, 
                # but nmethods_list is now property of class
                setattr(Class,'methods_list',ParentClass.methods_list)

    # return Class
    return Class

# def inherit_methods_list_from_parent(ParentClass,local_variables = locals()):
#     """function that given parent class, it either
#         1) creates methods_list key in locals
#         2) appends methods_list key to already existing methods_list in locals
#     """

#     frame = inspect.currentframe()
#     local_variables = frame.f_back.f_locals
 
#     if hasattr(ParentClass,"methods_list"):
#         if "methods_list" in local_variables.keys():
#             local_variables['methods_list'] += ParentClass.methods_list
#         else:
#             local_variables['methods_list'] = ParentClass.methods_list


def add_to_methods_list(func):
    """This is a decorator that takes a function name and it 
    appends this name to the list methods_list in locals
    """

    frame = inspect.currentframe()
    local_variables = frame.f_back.f_locals

    if 'methods_list' in local_variables.keys():
        # if methods_list already exists in locals, append function name
        local_variables['methods_list'].append(func.__name__)
    else:
        # if methods_list does not exist, create it and add function name
        local_variables['methods_list'] = [func.__name__]

    # return function
    return func