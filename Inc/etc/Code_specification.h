/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : Code_specification.h
 *  Description  : This document contains code specification requirements
 *  LastEditors  : Score
 *  Date         : 2022-04-17 08:29:09
 *  LastEditTime : 2022-04-17 08:34:12
 */
/*

No.1  Language usage standard
    Try to use English or English spelling for naming, and prohibit the use of Pinyin.
    Such as 
        Chassis 锟斤拷锟斤拷锟教ｏ拷
        Remote  锟斤拷遥锟斤拷锟斤拷锟斤拷
    You can't name it that way.
    Such as
        TUOLUO  锟斤拷锟斤拷锟捷ｏ拷
        YunTai  锟斤拷锟斤拷台锟斤拷
    
No.2 Naming standard
    Variable:
        1锟斤拷The global variables in the module are named by the format of "module name_variable name".
        2锟斤拷The module name is a word beginning with a capital letter, which should be consistent with the name of the file.
        3锟斤拷The variable name is a noun phrase beginning with a small letter. Except for the first word, each word begins with 
            a capital letter, and the variable function should be expressed with a concise modifier锟斤拷
        4锟斤拷Non global variables in the module are named in lowercase.
            what you need to take care is:
            The variable name does not need to include the module name.
            You can use abbreviations or incomplete words.
            If the variable name is multiple words, the words are separated by underscores.
            Single letter variable names are not recommended except for cyclic variables I, J, K, etc.
        Such as
            Motor_chasisMotor       // Chassis motor object, global variable
            Remote_remoteData       // Remote control data object, global variable
            ref                     // PID set point, local variable
    
    Const:
        1锟斤拷Macro definition should be avoided as far as possible in the module, and const definition 
           should be used instead. Constants are named using the "Const" module name.
        2锟斤拷The module name is a word beginning with a capital letter, which should be consistent with the name of the file.
        3锟斤拷The constant name is a noun phrase, each word is capitalized, the words are divided by underscores, and simple 
           modifiers are used to express the function.
        4锟斤拷Const in front of the name_ Prefix, indicating that the name is a constant name.
        Such as
            Const_ShooterLockedReverseSpeed   = -50.0f;     // Shooter motor locked reverse speed
        
    Custom type:
        struct:
        1锟斤拷The module name is used to name the structure type in the module_ Structure name "TypeDef".
        2锟斤拷The module name is a word beginning with a capital letter, which should be consistent with the name of the file.
        3锟斤拷The structure name is a noun phrase. Each word begins with a capital letter and uses a concise modifier to express its function.
        4锟斤拷If the structure name is followed by typedef, it means that this name is the structure type name.
        5锟斤拷The naming rules of structural elements are the same as that of local variables, which are separated by lowercase underscores.
        
        Enum:
        1锟斤拷Enumeration types within a module are named using the module name_ The format of the enumerator "Enum".
        2锟斤拷The module name is a word beginning with a capital letter, which should be consistent with the name of the file.
        3锟斤拷Enumerating names are noun phrases, each word begins with a capital letter, and uses simple modifiers to express its function.
        4锟斤拷Enum followed by enum indicates that the name is an enum type name.
        5锟斤拷Enumeration elements are named according to the same rules as constants, separated by capital letters and underscores.

        Such as
            Motor_MotorTypeDef      // Motor object struct
            Remote_RemoteStateEnum  // Remote state enum
    
    Function:
        1锟斤拷Functions are named using the "module name_ Function name".
        2锟斤拷The module name is a word beginning with a capital letter, which should be consistent with the name of the file.
        3锟斤拷The function name is a predicate phrase, each word is capitalized, and a concise modifier is used to express the 
           function function and data characteristics (get / change).
        4锟斤拷The naming rules of function parameters are the same as those of local variables.
        Such as
            Remote_ResetRemoteData();       // reset remote data object (function)
            pmotor_group                    // Motor group pointer (function parameter)

    File:
        1锟斤拷File are named using the "file content_ File level" format.
        2锟斤拷Files at the same level are placed in the same level folder.
        3锟斤拷Do not use Chinese or special characters锟斤拷contains PIN YIN).
        4锟斤拷File level try to ues abbreviations.
        Such as
            Utility->{ adc_uttil.c(h)
                     { can_util.c(h)
                     { dac_util.c(h)
                     { uart_util.c(h)

No.3 Notes 
    In line note requirements:
        You can use / * comment * /, or / / comment, but there should be a space between the comment number and the comment content.
    
    Variable declaration comment requirements:
        As above, the function and application scope of variables shall be indicated.

    Function declaration annotation requirements:
        Format:
*/
            /**
             * @brief      Function action
             * @param      Parameter 1: parameter function
             * @param      Parameter 2: parameter function
             * @retval     Function of return value
             */
/*
    File header annotation requirements:
        Format:
*/
            /**
             * Robomaster Source/Header File (Note: here is your name for the project, such as dreamcode, etc,
                Header file, source file code file)
             *
             * File: File name
             * Brief: File Function
             * Author: Author
             * Modified: Modify time or create time
             *
             */
/*

No.4 Writing standard
    Empty line:
        1锟斤拷Empty lines after defining variables.
        2锟斤拷Each function definition is followed by a blank line.
        3锟斤拷Add blank lines between functions of different levels in the same function.
        4锟斤拷Leave two empty lines between two function definitions.
        5锟斤拷The header comment and header file keep a blank line, 
           and a blank line between the header file and other definitions and functions.
        6锟斤拷ect
    
    Space锟斤拷
        1锟斤拷Leave a space after the keyword(lick锟斤拷const锟斤拷case).
        2锟斤拷For example, if, for, while and other keywords should be followed by a space and left parenthesis"(".
        3锟斤拷Do not leave a space after the function name, it should be followed by the left bracket "(", 
           to distinguish it from the keyword.
        4锟斤拷"," followed by a space. If ";" is not the end of a line, leave a space after it.
        5锟斤拷Assignment operators, relational operators, arithmetic operators, logical operators, 
           bit operators, such as =, =! =The following binocular operators, + =, - =, * =, / =,
           % =, > =, < < =, & =, ^ =, | =, >, < =, >, > =, +, -, *, /,%, &, |, <, >, ^, should 
           be preceded by spaces.
        6锟斤拷The operator "%" is a complement operator, which is different from the "%" of% d in printf, 
           so there is no need to add spaces before and after the "%" in %d.
        7锟斤拷Single operator!, ~, + +, -, -, *, & etc. without spaces before and after.
        8锟斤拷array symbol, struct member operator, and struct member operator - >, 
           there are no spaces before and after these operators.
        9锟斤拷
    
    Indent锟斤拷
        1锟斤拷Indentation is realized by tab on the keyboard, which can make the program more hierarchical. 
           The principle is that if the status is equal, indent is not required; if the internal code belongs to a code, 
           it needs to be indented.
        2锟斤拷Indents align units automatically. Do not mix indentation with space.
    
    Alignment:
        1锟斤拷"{}" in function, "{" needs to be aligned with the function named line and a space is reserved.
           "}" needs to have the last line alone.   
           Such as
           int main(void) {
               return 0;
           }
        2锟斤拷The code within {} should be indented by a tab, and those in the same position should be left aligned, 
           while those in different positions should continue to be indented.
           Such as
            int main(void) {
                if (锟斤拷) {
                    while (锟斤拷)
                }
                return 0;
            }
    Code line:
        1锟斤拷A line of code only does one thing, such as defining only one variable or writing only one statement.
        2锟斤拷If, else, for, while, do and other statements occupy one line, and the execution statement shall not be immediately followed.

No.5 Header file reference:
    1锟斤拷Add a macro definition in the header file to prevent the header file from repeatedly referencing the macro.
    2锟斤拷The header file reference should be referenced level by level as far as possible, and the lower level header .
       file and its lower level header file should not be referenced at the same time.
    3锟斤拷Do not refer to parent or peer header files in header files.
    4锟斤拷The parent or peer header file is referenced in the source file, and the lower level header file can be referenced in the header file.

*/
