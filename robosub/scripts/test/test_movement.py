"""Import direction"""
from modules.control.direction import Direction


def main():
    direction = Direction()

    print('turn left 90')
    direction.set_direction(-90, 0)
    direction.move_direction()

    print('move up 20')
    direction.set_direction(0, 20)
    direction.move_direction()

    print('turn right 180')
    direction.set_direction(180, 0)
    direction.move_direction()

    print('move down 40')
    direction.set_direction(0, -40)
    direction.move_direction()


if __name__ == '__main__':
    main()
