def callService(self,ServiceName,ServiceType,args):
    ParameterList = []
    if args:
        for i,arg in enumerate(args):
            exec('par'+str(i)+'=arg')
            ParameterList.append('par'+str(i))
    parameters = ','.join(ParameterList)
    
    try:
        rospy.wait_for_service(ServiceName)
        exec('servicecall = rospy.ServiceProxy(\''+ServiceName+'\',' +ServiceType+')')

    except rospy.ROSException:
        return None
        
    try:
        exec('req = '+ServiceType+'Request('+parameters+')')
        resp = servicecall(req)
        return resp

    except rospy.ServiceException:
        return None