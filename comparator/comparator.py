import json
import numpy as np
import matplotlib.pyplot as plt

def json2dict(filename):
    with open(filename) as handle:
        dictlist = json.loads(handle.read())
        return dictlist


def jsonComparatoraux(gt,op,joint1,joint2):
    lastidx = min(len(gt),len(op))
    errorarray = np.zeros(lastidx)

    for i in range(lastidx):
        #print i
        if len(gt[i]['keypoints']) == 0 and len(op[i]['keypoints']) == 0:
            errorarray[i] = -3
            continue
        elif len(gt[i]['keypoints']) == 0:
            errorarray[i] = -2
            continue
        elif len(op[i]['keypoints']) == 0:
            errorarray[i] = -1
            continue
        keyptsGT = [gt[i]['keypoints'][iaux]['partName'] if gt[i]['keypoints'][iaux] != None else ''
                     for iaux in range(len(gt[i]['keypoints']))]
                                                            
        keyptsOP = [op[i]['keypoints'][iaux]['partName'] if op[i]['keypoints'][iaux] != None else ''
                                                        for iaux in range(len(op[i]['keypoints']))]
        if (joint1 not in keyptsGT) and (joint2 not in keyptsOP):
            errorarray[i] = -3
            continue
        elif joint1 not in keyptsGT:
            errorarray[i] = -2
            continue
        elif joint2 not in keyptsOP:
            errorarray[i] = -1
            continue
        #print keyptsGT[keyptsGT.index(joint1)]
        #print gt[i]['keypoints'][keyptsGT.index(joint1)]['partName']
        #print gt[i]['keypoints'][keyptsGT.index(joint1)]['position']
        posxGT = gt[i]['keypoints'][keyptsGT.index(joint1)]['position']['x']
        posyGT = gt[i]['keypoints'][keyptsGT.index(joint1)]['position']['y']
        posxOP = op[i]['keypoints'][keyptsOP.index(joint2)]['position']['x']
        posyOP = op[i]['keypoints'][keyptsOP.index(joint2)]['position']['y']
        errorarray[i] = np.sqrt((posxGT-posxOP)**2 + (posyGT-posyOP)**2)
    return errorarray

    """
[u'torso', u'head', u'leftShoulder', u'leftElbow',
 u'leftHand', u'rightShoulder', u'rightElbow', u'rightHand', u'neck', u'leftHip',
 u'leftKnee', u'leftFoot', u'rightHip', u'rightKnee', u'rightFoot']
 
 [u'Nose', u'Neck', u'RShoulder', u'RElbow', u'RWrist',
  u'LShoulder', u'LElbow', u'LWrist', u'RHip', u'RKnee',
  u'RAnkle', u'LHip', u'LKnee', u'LAnkle', u'REye', u'LEye', u'LEar']

 """

def jsonComparator(gt,op):
    errorLShoulder = jsonComparatoraux(gt,op,'leftShoulder','LShoulder')
    errorRShoulder = jsonComparatoraux(gt,op,'rightShoulder','RShoulder')

    errorLElbow = jsonComparatoraux(gt,op,'leftElbow','LElbow')
    errorRElbow = jsonComparatoraux(gt,op,'rightElbow','RElbow')

    errorLHip = jsonComparatoraux(gt,op,'leftHip','LHip')
    errorRHip = jsonComparatoraux(gt,op,'rightHip','RHip')

    errorLKnee = jsonComparatoraux(gt,op,'leftKnee','LKnee')
    errorRKnee = jsonComparatoraux(gt,op,'rightKnee','RKnee')

    errorNeck = jsonComparatoraux(gt,op,'neck','Neck')

    kptserror = np.array([errorLShoulder, errorRShoulder,
                            errorLElbow, errorRElbow,
                            errorLHip, errorRHip,
                            errorLKnee, errorRKnee,
                            errorNeck])

    jointError = np.zeros(len(errorNeck))
    for frame_idx in range(len(errorNeck)):
        error = kptserror[:,frame_idx]
        error = error[error>=0]
        if len(error) != 0:
            jointError[frame_idx] = error.mean()
        else:
            jointError[frame_idx] = -1

    Emean = jointError[jointError>=0].mean()
    Estd = jointError[jointError>=0].std()
    return jointError, Emean, Estd, kptserror


def jsonComparator2(gt,op,idxlist):
    errorLShoulder = jsonComparatoraux(gt,op,'leftShoulder','LShoulder')
    errorRShoulder = jsonComparatoraux(gt,op,'rightShoulder','RShoulder')

    errorLElbow = jsonComparatoraux(gt,op,'leftElbow','LElbow')
    errorRElbow = jsonComparatoraux(gt,op,'rightElbow','RElbow')

    errorLHip = jsonComparatoraux(gt,op,'leftHip','LHip')
    errorRHip = jsonComparatoraux(gt,op,'rightHip','RHip')

    errorLKnee = jsonComparatoraux(gt,op,'leftKnee','LKnee')
    errorRKnee = jsonComparatoraux(gt,op,'rightKnee','RKnee')

    errorNeck = jsonComparatoraux(gt,op,'neck','Neck')

    kptserror = np.array([errorLShoulder, errorRShoulder,
                            errorLElbow, errorRElbow,
                            errorLHip, errorRHip,
                            errorLKnee, errorRKnee,
                            errorNeck])

    kptserror = kptserror[idxlist,:]

    jointError = np.zeros(len(errorNeck))
    for frame_idx in range(len(errorNeck)):
        error = kptserror[:,frame_idx]
        error = error[error>=0]
        if len(error) != 0:
            jointError[frame_idx] = error.mean()
        else:
            jointError[frame_idx] = -1

    Emean = jointError[jointError>=0].mean()
    Estd = jointError[jointError>=0].std()
    return jointError, Emean, Estd, kptserror


def firstidx(jsonlist):
    idx = 0
    for element in jsonlist:
        if len(element['keypoints']) != 0:
            return idx
        idx +=1


def errorSmoother(errorarray,frameRate):
    erroraux = np.zeros(len(errorarray)-frameRate+1)
    for idx in range(frameRate,len(errorarray)+1):
        aux = errorarray[idx-frameRate:idx]
        aux = aux[aux>=0]
        if len(aux) == 0:
            erroraux[idx-frameRate] = -1
        else:
            erroraux[idx-frameRate] = np.min(aux)
    return erroraux

"""
Kinect samples at 15Hz ~ 16Hz, in order to approximate 
the 4Hz sampling that we can afford with the performance of
a modern but affordable GPU we just must downsample from 15Hz
to 4Hz taking 1 every 4 samples.
"""
for seqidx in range(1,5):
    print str(seqidx)
    gtfile = '/json/groundTruth'+str(seqidx)+'.txt'
    opfile = '/json/openpose'+str(seqidx)+'.txt'

    gtlist = json2dict(gtfile)
    oplist = json2dict(opfile)

    #gtidx = firstidx(gtfile)
    #opidx = firstidx(opfile)

    #idxcomp = max(idxgt,idxop)

    ##full body
    jointError, Emean, Estd, kptsError = jsonComparator(gtlist,oplist)
    print 'full body'
    print 'Emean: '+str(Emean)
    print 'Estd: '+str(Estd)
    
    #pltindex = np.arange(len(jointError))[jointError>=0]
    #pltError = jointError[jointError>=0]
    pltindex = np.arange(len(jointError))
    pltError = jointError
    
    plt.plot(pltindex,pltError,'ro')
    plt.xlabel('Frame')
    plt.ylabel('Average Absolute Distance')
    plt.title('OpenPose vs. Ground Truth Seq '+str(seqidx)+': Whole Body')
    plt.savefig('wholebody_seq'+str(seqidx)+'.png')
    plt.close()

    ##smooth full body
    frameRate = 4
    smoothJointError = jointError[pltindex%frameRate == 0]
    smoothJointError = errorSmoother(smoothJointError,frameRate)

    print 'full body smooth'
    print 'Emean: '+str(smoothJointError.mean())
    print 'Estd: '+str(smoothJointError.std())

    pltindex = np.arange(len(jointError))
    pltindex = pltindex[pltindex%frameRate == 0][frameRate-1:]
    pltError = smoothJointError

    plt.plot(pltindex,pltError,'ro')
    plt.xlabel('Frame')
    plt.ylabel('Average Absolute Distance')
    plt.title('OpenPose vs. Ground Truth Seq '+str(seqidx)+': Best Whole Body 1Hz')
    plt.savefig('wholebody_1hz_seq'+str(seqidx)+'.png')
    plt.close()


    ##upper body
    idxlist = [0,1,2,3,4,5,8]
    jointError, Emean, Estd, kptsError = jsonComparator2(gtlist,oplist,idxlist)
    print 'upper body'
    print 'Emean: '+str(Emean)
    print 'Estd: '+str(Estd)

    #pltindex = np.arange(len(jointError))[jointError>=0]
    #pltError = jointError[jointError>=0]
    pltindex = np.arange(len(jointError))
    pltError = jointError

    plt.plot(pltindex,pltError,'ro')
    plt.xlabel('Frame')
    plt.ylabel('Average Absolute Distance')
    plt.title('OpenPose vs. Ground Truth Seq '+str(seqidx)+': Upper Body')
    plt.savefig('upperbody_seq'+str(seqidx)+'.png')
    plt.close()

    ##smooth upperbody
    frameRate = 4
    smoothJointError = jointError[pltindex%frameRate == 0]
    smoothJointError = errorSmoother(smoothJointError,frameRate)
    
    print 'upper body smooth'
    print 'Emean: '+str(smoothJointError.mean())
    print 'Estd: '+str(smoothJointError.std())

    pltindex = np.arange(len(jointError))
    pltindex = pltindex[pltindex%frameRate == 0][frameRate-1:]
    pltError = smoothJointError

    plt.plot(pltindex,pltError,'ro')
    plt.xlabel('Frame')
    plt.ylabel('Average Absolute Distance')
    plt.title('OpenPose vs. Ground Truth Seq '+str(seqidx)+': Best UpperBody 1Hz')
    plt.savefig('upperbody_1hz_seq'+str(seqidx)+'.png')
    plt.close()

