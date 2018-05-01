from houston import Houston

houston = Houston()

print('testing start')
houston.start()
houston.do_task('gate')
for i in range(1, 100):
    houston.do_task('gate')