using System.Collections;
using UnityEngine;
using UnityEngine.SceneManagement;

public class StartMenu : MonoBehaviour
{
    public Animator animator;
    public AnimationClip startGameClip;
    public void StartGame()
    {
        StartCoroutine(StartGameIEnum());
    }
    IEnumerator StartGameIEnum()
    {
        animator.Play(startGameClip.name);
        yield return new WaitForSeconds(startGameClip.length);
        SceneManager.LoadScene("track1");
    }
}
