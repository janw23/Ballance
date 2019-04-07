if __name__ == "__main__":
    import sys
    import pstats
    import io

    s = io.StringIO()

    stats = pstats.Stats("/home/pi/ballance/Ballance/PythonCode/stats", stream=s)
    stats.strip_dirs()
    stats.sort_stats('tottime')
    stats.print_stats()
    
    with open('ballancestats.txt', 'w+') as f:
        f.write(s.getvalue())