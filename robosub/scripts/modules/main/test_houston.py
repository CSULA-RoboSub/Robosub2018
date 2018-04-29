from houston import Houston

houston = Houston()

print('testing start')
houston.start()

for i in range(1, 100):
    houston.do_task('gate')