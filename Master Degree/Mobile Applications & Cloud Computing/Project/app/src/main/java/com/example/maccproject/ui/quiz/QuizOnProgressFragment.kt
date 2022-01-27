package com.example.maccproject.ui.quiz


import android.os.Bundle
import android.view.*
import android.widget.ImageButton
import android.widget.TextView
import androidx.fragment.app.Fragment
import androidx.navigation.findNavController
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentQuizOnProgressBinding
import android.widget.Toast
import androidx.activity.OnBackPressedCallback
import androidx.core.os.bundleOf
import androidx.navigation.fragment.findNavController


class QuizOnProgressFragment : Fragment() {
    data class Question(
        val text: String,
        val answers: List<String>)

    // The first answer is the correct one.  We randomize the answers before showing the text.
    // All questions must have four answers.  We'd want these to contain references to string
    // resources so we could internationalize. (or better yet, not define the questions in code...)
    private val questions: MutableList<Question> = mutableListOf(
        Question(
            text = "Which of the two characters fascinates you the most?",
            answers = listOf("quiz1", "quiz2")
        ),
        Question(
            text = "If you could fly a spaceship, which of the two fighters would you choose?",
            answers = listOf("quiz3", "quiz4")
        ),
        Question(
            text = "Would you rather be part of the Solo-Chewba gang or would you rather be one " +
                    "of the most powerful, strong and privileged imperial guards?",
            answers = listOf("quiz5", "quiz6")
        ),
        Question(
            text = "Would you rather be Yoda, esteemed Jedi master but limited in his powers " +
                    "and actions by the Republic, or Palpatine, supreme and all-powerful Sith dedicated " +
                    "to the cause of the empire?",
            answers = listOf("quiz7", "quiz8")
        ),
        Question(
            text = "You are the new interstellar admiral. Is your ship the glamorous Falcon or " +
                    "the legendary Death Star?",
            answers = listOf("quiz9", "quiz10")
        ),
    )

    lateinit var currentQuestion: Question
    lateinit var answers: MutableList<String>
    private var questionIndex = 0
    private val numTotalQuestions = questions.count()-1
    private lateinit var image1: ImageButton
    private lateinit var image2: ImageButton
    private var checkClicked = -1
    private var accumulatedPoints = 0

    private var _binding: FragmentQuizOnProgressBinding? = null
    private val binding get() = _binding!!

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
                              savedInstanceState: Bundle?): View {

        // Inflate the layout for this fragment
        _binding = FragmentQuizOnProgressBinding.inflate(inflater, container, false)

        val root: View = binding.root

        image1 = binding.imageQuiz1
        image2 = binding.imageQuiz2

        //To enabled the back button
        setHasOptionsMenu(true)
        // Shuffles the questions and sets the question index to the first question.
        randomizeQuestions()

        // Bind this fragment class to the layout
        binding.quiz = this

        // Set the onClickListener for the submitButton
        binding.quizSubmitButton.setOnClickListener @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        { view: View ->
            if(checkClicked!=-1) {
                // se la risposta scelta è uguale al primo elemento delle risposte nelle domande non mescolate, allora hai un punto
                if(questions[questionIndex].answers[0]==answers[checkClicked]){
                    accumulatedPoints+=1
                }
                checkClicked=-1
                if(questionIndex<numTotalQuestions){
                    //Changing Questions
                    questionIndex++
                    currentQuestion = questions[questionIndex]
                    setQuestion()
                    binding.invalidateAll()
                } else {
                    val bundle = bundleOf("accumulatedPoints" to accumulatedPoints)
                    view.findNavController().navigate(R.id.action_quiz_graphic_to_quiz_result, bundle)
                }
            }else{
                Toast.makeText(activity?.applicationContext,
                    "Before submitting, you have to make a choice" , Toast.LENGTH_SHORT).show()
            }

        }

        // Set the onClickListener for the first image
        binding.imageQuiz1.setOnClickListener @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        { view: View ->
            checkClicked = 0
            binding.imageQuiz1.setBackgroundResource(R.drawable.bordergreen)
            binding.imageQuiz2.setBackgroundResource(R.drawable.bordertrasparent)
        }

        // Set the onClickListener for the second image
        binding.imageQuiz2.setOnClickListener @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        { view: View ->
            checkClicked = 1
            binding.imageQuiz1.setBackgroundResource(R.drawable.bordertrasparent)
            binding.imageQuiz2.setBackgroundResource(R.drawable.bordergreen)
        }

        // Handle button back on the top menu
        activity?.onBackPressedDispatcher?.addCallback(viewLifecycleOwner, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                // in here you can do logic when backPress is clicked
                root.findNavController().navigate(R.id.action_quiz_graphic_to_navigation_quiz)
            }
        })

        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    private fun randomizeQuestions() {
        questions.shuffle()
        questionIndex = 0
        setQuestion()
    }

    private fun setQuestion() {
        image1.setBackgroundResource(R.drawable.bordertrasparent)
        image2.setBackgroundResource(R.drawable.bordertrasparent)
        currentQuestion = questions[questionIndex]
        // randomize the answers into a copy of the array
        answers = currentQuestion.answers.toMutableList()
        // and shuffle them
        answers.shuffle()
        val textQuestionBox: TextView = binding.questionBox
        textQuestionBox.text = currentQuestion.text

        when(answers[0]){
            "quiz1" -> image1.setImageResource(R.drawable.quiz1)
            "quiz2" -> image1.setImageResource(R.drawable.quiz2)
            "quiz3" -> image1.setImageResource(R.drawable.quiz3)
            "quiz4" -> image1.setImageResource(R.drawable.quiz4)
            "quiz5" -> image1.setImageResource(R.drawable.quiz5)
            "quiz6" -> image1.setImageResource(R.drawable.quiz6)
            "quiz7" -> image1.setImageResource(R.drawable.quiz7)
            "quiz8" -> image1.setImageResource(R.drawable.quiz8)
            "quiz9" -> image1.setImageResource(R.drawable.quiz9)
            "quiz10" -> image1.setImageResource(R.drawable.quiz10)
        }
        when(answers[1]){
            "quiz1" -> image2.setImageResource(R.drawable.quiz1)
            "quiz2" -> image2.setImageResource(R.drawable.quiz2)
            "quiz3" -> image2.setImageResource(R.drawable.quiz3)
            "quiz4" -> image2.setImageResource(R.drawable.quiz4)
            "quiz5" -> image2.setImageResource(R.drawable.quiz5)
            "quiz6" -> image2.setImageResource(R.drawable.quiz6)
            "quiz7" -> image2.setImageResource(R.drawable.quiz7)
            "quiz8" -> image2.setImageResource(R.drawable.quiz8)
            "quiz9" -> image2.setImageResource(R.drawable.quiz9)
            "quiz10" -> image2.setImageResource(R.drawable.quiz10)
        }

    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        when (item.itemId) {
            android.R.id.home -> findNavController().navigate(R.id.action_quiz_graphic_to_navigation_quiz)
        }
        return true
    }

}
