import Test

def main():
    me = Test.Test("Josiah")
    me.age = 21
    me.major = "Computer Science"
    me.cat = Test.Test("Tibbers")
    me.cat.age = 32
    me.createLogger('rotating_file.log')

main()
