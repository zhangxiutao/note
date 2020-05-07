def r(n,path,l):
    if l == -1:
        print(path)
        return
    for i in range(1,n+1):
        path[l] = i
        if path[l] in path[l+1:]:
            continue
        r(n,path,l-1)
p = [0,0,0]
r(3,p,2)