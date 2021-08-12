import math
import fileinput
import sys
import re
import os

def is_header_comment(line):
    if "//" in line and ">>" in line:
        return True
    return False

def is_summary_comment(line):
    if "///" in line:
        return True
    return False

def get_tabs(line):
    return line.count("    ")

def assign_index(indecies, tabs):
    if tabs > len(indecies):
        while tabs > len(indecies):
            num_ind = (len(indecies) % 3)
            if num_ind == 0:
                indecies.append("1")
            elif num_ind == 1:
                indecies.append("a")
            elif num_ind == 2:
                indecies.append("i")
    elif tabs < len(indecies):
        while tabs < len(indecies):
            indecies.pop()
        iter_index(indecies)
    else:
        iter_index(indecies)

def iter_index(indecies):
    if len(indecies) % 3 == 1:
        indecies[-1] = str(int(indecies[-1]) + 1)
    elif len(indecies) % 3 == 2:
        value = 0
        letter_len = len(indecies[-1]) - 1
        index_value = ""
        for letter in indecies[-1]:
            value = value + (ord(letter) - 96) * (26 ** letter_len)
            letter_len = letter_len - 1
        value = value + 1
        digits = 1
        while value > 26 ** digits:
            digits = digits + 1
        while value > 0:
            index_value = chr((value % 26) + 96) + index_value
            value = math.floor(value / 26)
        indecies[-1] = index_value
    else:
        if len(indecies) == 0:
            return
        romanNum = ""
        roman = {'i':1,'v':5,'x':10,'l':50,'c':100,'d':500,'m':1000,'iv':4,'ix':9,'xl':40,'xl':90,'cd':400,'cm':900}
        i = 0
        num = 0
        while i < len(indecies[-1]):
            if i+1<len(indecies[-1]) and indecies[-1][i:i+2] in roman:
                num+=roman[indecies[-1][i:i+2]]
                i+=2
            else:
                #print(i)
                num+=roman[indecies[-1][i]]
                i+=1
        num = num + 1

        intToroman = { 1: 'i', 4: 'iv', 5: 'v', 9: 'ix', 10: 'x', 40: 'xl', 50: 'l', 90: 'xc', 100: 'c', 400: 'xd', 500: 'd', 900: 'cm', 1000: 'm'}
        print_order = [1000, 900, 500, 400, 100, 90, 50, 40, 10, 9, 5, 4, 1]

        integer = num

        for x in print_order:
            if integer != 0:
                quotient= integer//x

                #If quotient is not zero output the roman equivalent
                if quotient != 0:
                    for y in range(quotient):
                        romanNum = romanNum + intToroman[x]
                        #print(intToroman[x], end=""

                        #update integer with remainder
                        integer = integer%x
        indecies[-1] = romanNum

def add_index(line, index):
    comment_sign_start = line.find("//")
    line_start = line[:comment_sign_start + 2]
    line_end = line[comment_sign_start + 2:]
    if index != "":
        return (line_start + " " + index + line_end)
    return (line_start + index + line_end)

def concat_index(indecies):
    index = ""
    for numeral in indecies:
        if index == "":
            index = index + numeral
        else:
            index = index + "." + numeral
    return index

def create_banner(file_name, max_len=60):
    banner = "********** " + file_name + " "
    return banner + ((max_len-4)-len(banner))*"*"

def insert_summary(file_name, header_comments):
    summary_line = 0
    non_includes = 0
    curr_line = 0
    with open(file_name) as fp:
        line = fp.readline()
        curr_line += 1
        while line:
            if non_includes == 2:
                break
            if "#include" not in line and "#define" not in line and "#ifndef" not in line:
                non_includes += 1
                line = fp.readline()
                curr_line += 1
            else:
                non_includes = 0
                summary_line = curr_line
                line = fp.readline()
                curr_line += 1
    #print("Summary Line: " + str(summary_line))
    summary_line += 1

    contents = []

    with open(file_name, "r") as f:
        contents = f.readlines()

    for i in range(len(header_comments)):
        header_comments[i] = re.sub('// ', '', header_comments[i])
        header_comments[i] = re.sub('>>', '', header_comments[i])
        header_comments[i] = "///"+header_comments[i]
        contents.insert(summary_line + i, header_comments[i])

    with open(file_name, "w") as f:
        f.writelines(contents)

def main():
    rootdir = './src'
    for subdir, dirs, files in os.walk(rootdir):
        for file in files:
            if len(sys.argv) > 1:
                if file == sys.argv[1]:
                    indecies = []
                    header_comments = []
                    new_line = ""
                    max_comment_len = -1
                    file_name = file
                    last_line_included = False
                    last_line_index_len = -1
                    line_num = 1
                    for line in fileinput.input(os.path.join(subdir, file), inplace=True): #sys.argv[1]
                        if is_header_comment(line):
                            if last_line_included:
                                line = re.sub('//.*?>>', '// >>', line)
                                print(line, end='')
                                line = " "*(last_line_index_len+1) + line
                                line = "/////" + line
                                header_comments[len(header_comments)-1] = header_comments[len(header_comments)-1] + line
                            else:
                                line = re.sub('//.*?>>', '// >>', line)
                                assign_index(indecies, get_tabs(line))
                                last_line_index_len = len(line)
                                new_line = add_index(line, concat_index(indecies))
                                last_line_index_len = len(new_line) - last_line_index_len
                                print(new_line, end='')
                                new_line = new_line[:-1] + " | " + str(line_num) + "\n"
                                header_comments.append(new_line)
                            last_line_included = True
                        elif is_summary_comment(line):
                            print("", end='')
                            last_line_included = False
                        else:
                            print(line, end='')
                            last_line_included = False
                        line_num += 1

                    if len(header_comments) > 0:
                        for comment in header_comments:
                            if len(comment) > max_comment_len:
                                max_comment_len = len(comment)

                        if max_comment_len < (len(file_name) + 12):
                            header_comments.insert(0, create_banner(file_name) + "\n")
                            header_comments.append((len(header_comments[1])-1)*"*" + "\n")
                        else:
                            header_comments.insert(0, create_banner(file_name, max_comment_len) + "\n")
                            header_comments.append((max_comment_len-4)*"*" + "\n")

                        for comment in header_comments:
                            comment = re.sub('// ', '', comment)
                            comment = re.sub('>>', '', comment)
                            if len(comment) != 1:
                                comment = "///"+comment

                        insert_summary(os.path.join(subdir, file), header_comments)
            else:
                if ".cpp" in file or ".hpp" in file:
                        indecies = []
                        header_comments = []
                        new_line = ""
                        max_comment_len = -1
                        file_name = file
                        last_line_included = False
                        last_line_index_len = -1
                        line_num = 1
                        for line in fileinput.input(os.path.join(subdir, file), inplace=True): #sys.argv[1]
                            if is_header_comment(line):
                                if last_line_included:
                                    line = re.sub('//.*?>>', '// >>', line)
                                    print(line, end='')
                                    line = " "*(last_line_index_len+1) + line
                                    line = "/////" + line
                                    header_comments[len(header_comments)-1] = header_comments[len(header_comments)-1] + line
                                else:
                                    line = re.sub('//.*?>>', '// >>', line)
                                    assign_index(indecies, get_tabs(line))
                                    last_line_index_len = len(line)
                                    new_line = add_index(line, concat_index(indecies))
                                    last_line_index_len = len(new_line) - last_line_index_len
                                    print(new_line, end='')
                                    new_line = new_line[:-1] + " | " + str(line_num) + "\n"
                                    header_comments.append(new_line)
                                last_line_included = True
                            elif is_summary_comment(line):
                                print("", end='')
                                last_line_included = False
                            else:
                                print(line, end='')
                                last_line_included = False
                            line_num += 1

                        if len(header_comments) > 0:
                            for comment in header_comments:
                                if len(comment) > max_comment_len:
                                    max_comment_len = len(comment)

                            if max_comment_len < (len(file_name) + 12):
                                header_comments.insert(0, create_banner(file_name) + "\n")
                                header_comments.append((len(header_comments[1])-1)*"*" + "\n")
                            else:
                                header_comments.insert(0, create_banner(file_name, max_comment_len) + "\n")
                                header_comments.append((max_comment_len-4)*"*" + "\n")

                            for comment in header_comments:
                                comment = re.sub('// ', '', comment)
                                comment = re.sub('>>', '', comment)
                                if len(comment) != 1:
                                    comment = "///"+comment

                            insert_summary(os.path.join(subdir, file), header_comments)

main()